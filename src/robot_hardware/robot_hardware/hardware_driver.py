#!/usr/bin/env python3
"""
hardware_driver — ROS 2 bridge node for differential-drive tracked robot.

Communicates with an Arduino Mega over USB serial and bridges:
  - Encoder ticks  → /odom (nav_msgs/Odometry) + TF odom→base_link
  - IMU raw data   → /imu/data_raw (sensor_msgs/Imu)
  - /cmd_vel       → serial CMD to Arduino

Serial protocol (115200 baud):
  Arduino → Pi (20 Hz):  "DATA,<tL>,<tR>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n"
  Pi → Arduino:          "CMD,<v_left>,<v_right>\n"
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import serial

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import (
    Twist,
    TransformStamped,
    Quaternion,
)
from tf2_ros import TransformBroadcaster


# ── helpers ──────────────────────────────────────────────────────────────────

def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a Quaternion message from a yaw angle (radians)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ── main node ────────────────────────────────────────────────────────────────

class HardwareDriverNode(Node):
    """Bridge between Arduino Mega serial and ROS 2 topics."""

    def __init__(self):
        super().__init__('hardware_driver')

        # ── declare parameters ───────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout', 0.01)

        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheelbase', 0.22)
        self.declare_parameter('encoder_ppr', 3960)
        self.declare_parameter('max_speed', 0.3)

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'imu_link')

        self.declare_parameter('publish_tf', True)
        self.declare_parameter('cmd_vel_timeout', 1.0)

        # ── read parameters ──────────────────────────────────────────────
        self._port = self.get_parameter('serial_port').value
        self._baud = self.get_parameter('baudrate').value
        self._ser_timeout = self.get_parameter('serial_timeout').value

        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._wheelbase = self.get_parameter('wheelbase').value
        self._encoder_ppr = self.get_parameter('encoder_ppr').value
        self._max_speed = self.get_parameter('max_speed').value

        self._odom_frame = self.get_parameter('odom_frame_id').value
        self._base_frame = self.get_parameter('base_frame_id').value
        self._imu_frame = self.get_parameter('imu_frame_id').value

        self._publish_tf = self.get_parameter('publish_tf').value
        self._cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # ── derived constants ────────────────────────────────────────────
        # Distance per encoder tick (meters)
        self._meters_per_tick = (
            2.0 * math.pi * self._wheel_radius / self._encoder_ppr
        )

        # ── odometry state ───────────────────────────────────────────────
        self._x = 0.0       # metres in odom frame
        self._y = 0.0
        self._theta = 0.0   # radians

        self._prev_ticks_l = None   # set on first DATA packet
        self._prev_ticks_r = None
        self._prev_stamp = None     # rclpy.time.Time

        # ── serial port ─────────────────────────────────────────────────
        self._serial: serial.Serial | None = None
        self._serial_lock = threading.Lock()
        self._open_serial()
        self._last_reconnect_attempt = self.get_clock().now()

        # ── publishers ───────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        # ── TF broadcaster ───────────────────────────────────────────────
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        # ── subscriber ───────────────────────────────────────────────────
        self._last_cmd_vel_time = self.get_clock().now()
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )

        # ── timer — 20 Hz read loop ─────────────────────────────────────
        self._timer = self.create_timer(0.05, self._timer_callback)

        self.get_logger().info(
            f'hardware_driver started — serial {self._port}@{self._baud}'
        )

    # ─────────────────────────────────────────────────────────────────────
    # Serial helpers
    # ─────────────────────────────────────────────────────────────────────

    def _open_serial(self):
        """Open (or reopen) the serial port."""
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                timeout=self._ser_timeout,
            )
            # Flush any stale data sitting in the Arduino's boot-up buffer
            self._serial.reset_input_buffer()
            self.get_logger().info(f'Serial port {self._port} opened.')
        except serial.SerialException as exc:
            self._serial = None
            self.get_logger().error(
                f'Cannot open serial port {self._port}: {exc}'
            )

    def _write_serial(self, data: str):
        """Thread-safe write to serial port."""
        with self._serial_lock:
            if self._serial is not None and self._serial.is_open:
                try:
                    self._serial.write(data.encode('ascii'))
                except serial.SerialException as exc:
                    self.get_logger().warn(f'Serial write error: {exc}')
                    self._serial = None  # will attempt reopen

    def _readline_serial(self) -> str | None:
        """Non-blocking readline from serial.  Returns decoded string or None."""
        with self._serial_lock:
            if self._serial is None or not self._serial.is_open:
                return None
            try:
                raw = self._serial.readline()
                if raw:
                    return raw.decode('ascii', errors='ignore').strip()
            except serial.SerialException as exc:
                self.get_logger().warn(f'Serial read error: {exc}')
                self._serial = None
        return None

    # ─────────────────────────────────────────────────────────────────────
    # /cmd_vel subscriber callback
    # ─────────────────────────────────────────────────────────────────────

    def _cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist (v, ω) to individual wheel velocities (m/s) and
        send CMD to Arduino.

        Differential drive kinematics:
            v_left  = v - ω * L / 2
            v_right = v + ω * L / 2
        """
        v = msg.linear.x
        omega = msg.angular.z

        v_left = v - omega * self._wheelbase / 2.0
        v_right = v + omega * self._wheelbase / 2.0

        # Clamp to max speed
        v_left = max(-self._max_speed, min(self._max_speed, v_left))
        v_right = max(-self._max_speed, min(self._max_speed, v_right))

        cmd = f'CMD,{v_left:.4f},{v_right:.4f}\n'
        self._write_serial(cmd)

        self._last_cmd_vel_time = self.get_clock().now()

    # ─────────────────────────────────────────────────────────────────────
    # Timer callback — read serial & publish
    # ─────────────────────────────────────────────────────────────────────

    def _timer_callback(self):
        """Called at 20 Hz.  Read serial line(s) and publish odom + IMU."""

        # Attempt to reopen serial if it was lost (throttled to once per 5 s)
        if self._serial is None:
            _now = self.get_clock().now()
            _dt = (_now - self._last_reconnect_attempt).nanoseconds * 1e-9
            if _dt >= 5.0:
                self._last_reconnect_attempt = _now
                self._open_serial()
            return

        # ── cmd_vel timeout safety ───────────────────────────────────────
        now = self.get_clock().now()
        dt_cmd = (now - self._last_cmd_vel_time).nanoseconds * 1e-9
        if dt_cmd > self._cmd_vel_timeout:
            # Send zero velocity to stop motors
            self._write_serial('CMD,0.0000,0.0000\n')

        # ── read all available lines, process the latest ─────────────────
        latest_line: str | None = None
        for _ in range(10):  # drain up to 10 buffered lines
            line = self._readline_serial()
            if line is None or len(line) == 0:
                break
            if line.startswith('DATA,'):
                latest_line = line

        if latest_line is None:
            return

        # ── parse DATA packet ────────────────────────────────────────────
        try:
            parts = latest_line.split(',')
            if len(parts) != 9:
                self.get_logger().warn(
                    f'Malformed DATA packet ({len(parts)} fields): '
                    f'{latest_line[:60]}'
                )
                return

            ticks_l = int(parts[1])
            ticks_r = int(parts[2])
            ax = float(parts[3])
            ay = float(parts[4])
            az = float(parts[5])
            gx = float(parts[6])
            gy = float(parts[7])
            gz = float(parts[8])

        except (ValueError, IndexError) as exc:
            self.get_logger().warn(f'DATA parse error: {exc}')
            return

        stamp = self.get_clock().now().to_msg()

        # ── publish IMU ──────────────────────────────────────────────────
        self._publish_imu(stamp, ax, ay, az, gx, gy, gz)

        # ── compute & publish odometry ───────────────────────────────────
        self._update_odometry(stamp, ticks_l, ticks_r)

    # ─────────────────────────────────────────────────────────────────────
    # IMU publisher
    # ─────────────────────────────────────────────────────────────────────

    def _publish_imu(self, stamp, ax, ay, az, gx, gy, gz):
        """Publish sensor_msgs/Imu with raw accelerometer and gyroscope."""
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._imu_frame

        # Orientation is not provided by raw MPU6050 data — mark unknown
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0
        # covariance -1 ⇒ orientation data not available (REP-145)
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        # Diagonal covariance — tune after real calibration
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1

        self._imu_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────
    # Odometry computation (differential drive forward kinematics)
    # ─────────────────────────────────────────────────────────────────────

    def _update_odometry(self, stamp, ticks_l: int, ticks_r: int):
        """
        Compute 2-D pose from encoder tick deltas using exact arc integration.

        Sign convention (matching firmware):
          - LEFT  encoder: positive ticks = forward motion
          - RIGHT encoder: positive ticks = forward motion
        """

        now_time = Time.from_msg(stamp)

        # First call — initialise baseline and return
        if self._prev_ticks_l is None:
            self._prev_ticks_l = ticks_l
            self._prev_ticks_r = ticks_r
            self._prev_stamp = now_time
            return

        # ── delta ticks & time ───────────────────────────────────────────
        delta_l = ticks_l - self._prev_ticks_l
        delta_r = ticks_r - self._prev_ticks_r
        dt = (now_time - self._prev_stamp).nanoseconds * 1e-9

        self._prev_ticks_l = ticks_l
        self._prev_ticks_r = ticks_r
        self._prev_stamp = now_time

        if dt <= 0.0:
            return

        # ── wheel displacements (metres) ─────────────────────────────────
        dist_l = delta_l * self._meters_per_tick
        dist_r = delta_r * self._meters_per_tick

        # ── robot displacement (exact arc) ───────────────────────────────
        d_center = (dist_l + dist_r) / 2.0
        d_theta = (dist_r - dist_l) / self._wheelbase

        if abs(d_theta) < 1e-6:
            # Straight-line motion — avoid division by near-zero
            dx = d_center * math.cos(self._theta)
            dy = d_center * math.sin(self._theta)
        else:
            # Exact arc integration
            radius = d_center / d_theta
            dx = radius * (math.sin(self._theta + d_theta) - math.sin(self._theta))
            dy = -radius * (math.cos(self._theta + d_theta) - math.cos(self._theta))

        self._x += dx
        self._y += dy
        self._theta += d_theta
        # Normalise theta to [-π, π]
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        # ── velocities ───────────────────────────────────────────────────
        v_linear = d_center / dt
        v_angular = d_theta / dt

        # ── build Odometry message ───────────────────────────────────────
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame

        # Pose
        odom_msg.pose.pose.position.x = self._x
        odom_msg.pose.pose.position.y = self._y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion_from_yaw(self._theta)

        # Pose covariance (6×6, row-major):
        #   [x, y, z, roll, pitch, yaw]
        # Diagonal only; tune after characterization
        odom_msg.pose.covariance[0]  = 0.01   # x
        odom_msg.pose.covariance[7]  = 0.01   # y
        odom_msg.pose.covariance[14] = 1e6    # z   (not measured)
        odom_msg.pose.covariance[21] = 1e6    # roll
        odom_msg.pose.covariance[28] = 1e6    # pitch
        odom_msg.pose.covariance[35] = 0.03   # yaw

        # Twist (in child frame = base_link)
        odom_msg.twist.twist.linear.x = v_linear
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = v_angular

        odom_msg.twist.covariance[0]  = 0.01   # vx
        odom_msg.twist.covariance[7]  = 1e6    # vy  (non-holonomic)
        odom_msg.twist.covariance[14] = 1e6    # vz
        odom_msg.twist.covariance[21] = 1e6    # ωx
        odom_msg.twist.covariance[28] = 1e6    # ωy
        odom_msg.twist.covariance[35] = 0.03   # ωz

        self._odom_pub.publish(odom_msg)

        # ── TF broadcast ─────────────────────────────────────────────────
        if self._publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self._odom_frame
            tf_msg.child_frame_id = self._base_frame

            tf_msg.transform.translation.x = self._x
            tf_msg.transform.translation.y = self._y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = quaternion_from_yaw(self._theta)

            self._tf_broadcaster.sendTransform(tf_msg)

    # ─────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        """Send stop command and close serial on shutdown."""
        self.get_logger().info('Shutting down — sending stop command.')
        self._write_serial('CMD,0.0000,0.0000\n')
        with self._serial_lock:
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
        super().destroy_node()


# ── entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HardwareDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
