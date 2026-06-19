"""
coordinator.py — Interactive Waypoint + Anomaly Coordinator
=============================================================
Mode: Static Map + AMCL + Interactive RViz Waypoint Selection

Workflow (for defense demo):
  1. Launch system → RViz shows the map
  2. Operator uses RViz "Publish Point" tool to click waypoints on the map
     → Each click adds a waypoint (shown as green marker on map)
  3. Operator sends start signal:
     ros2 topic pub /start_inspection std_msgs/String "data: start" --once
  4. Robot autonomously navigates through ALL waypoints in order
  5. At each waypoint: pause 3s for YOLO camera scan
  6. If anomaly detected mid-transit: stop → mark red → resume
  7. After all waypoints: return to origin (0,0) → generate report

Topics:
  Subscribed:
    /clicked_point      (PointStamped)  — from RViz "Publish Point" tool
    /start_inspection   (String)        — trigger to begin autonomous run
    /anomaly            (String, JSON)  — from anomaly_detector
  Published:
    /cmd_vel            (Twist)         — emergency stop
    /anomaly_markers    (MarkerArray)   — red cylinders for anomalies
    /waypoint_markers   (MarkerArray)   — green spheres for planned waypoints
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Twist, PoseStamped, PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json
import math
import threading
import os
import time
from datetime import datetime


class AnomalyCoordinator(Node):
    def __init__(self):
        super().__init__('anomaly_coordinator')

        self.is_paused = False
        self.marker_id = 0
        self._lock = threading.Lock()
        self.detected_anomalies = []
        self._report_generated = False

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('save_dir', '/home/pi/robot_data/anomalies')
        self._save_dir = self.get_parameter('save_dir').value

        # ── Waypoint collection ──────────────────────────────────────────
        self.waypoints = []              # List of (x, y) tuples
        self._current_wp_idx = 0
        self._current_goal_handle = None
        self._state = "COLLECTING"       # COLLECTING, NAVIGATING, WAITING, COMPLETED
        self._home_x = 0.0
        self._home_y = 0.0

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_anomaly_marker = self.create_publisher(
            MarkerArray, '/anomaly_markers', 10)
        self.pub_waypoint_marker = self.create_publisher(
            MarkerArray, '/waypoint_markers', 10)
        self.pub_servo = self.create_publisher(
            Int32MultiArray, '/cmd_servo', 10)

        # ── Subscribers ──────────────────────────────────────────────────
        # RViz "Publish Point" tool → collect waypoints
        self.create_subscription(
            PointStamped, '/clicked_point', self._clicked_point_cb, 10)
        # Start trigger
        self.create_subscription(
            String, '/start_inspection', self._start_inspection_cb, 10)
        # Anomaly alerts from YOLO detector
        self.create_subscription(
            String, '/anomaly', self.anomaly_callback, 10)

        # ── Nav2 Action Client ───────────────────────────────────────────
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self.anomaly_marker_array = MarkerArray()
        self.waypoint_marker_array = MarkerArray()

        self.get_logger().info(
            '═══════════════════════════════════════════════════')
        self.get_logger().info(
            '  🗺️  WAYPOINT COLLECTION MODE')
        self.get_logger().info(
            '  Use RViz "Publish Point" tool to click waypoints')
        self.get_logger().info(
            '  Then send: ros2 topic pub /start_inspection \\')
        self.get_logger().info(
            '    std_msgs/String "data: start" --once')
        self.get_logger().info(
            '═══════════════════════════════════════════════════')

    def send_servo_cmd(self, pan: int, tilt: int):
        """Publish a command to rotate the camera Pan-Tilt."""
        msg = Int32MultiArray()
        msg.data = [int(pan), int(tilt)]
        self.pub_servo.publish(msg)

    # ------------------------------------------------------------------
    # Waypoint collection from RViz
    # ------------------------------------------------------------------

    def _clicked_point_cb(self, msg: PointStamped):
        """Handle clicks from RViz 'Publish Point' tool."""
        if self._state != "COLLECTING":
            self.get_logger().warn(
                'Inspection already started. Ignoring new waypoint.')
            return

        x = msg.point.x
        y = msg.point.y
        self.waypoints.append((x, y))

        # Rebuild and show markers + connected path line
        self._update_waypoint_markers()

        self.get_logger().info(
            f'📌 Waypoint {len(self.waypoints)} added: ({x:.2f}, {y:.2f})')

    def _update_waypoint_markers(self):
        """Rebuild and publish the entire waypoint marker array (spheres, texts, and path line)."""
        self.waypoint_marker_array.markers.clear()

        # 1. Add Sphere and Text for each waypoint
        for i, wp in enumerate(self.waypoints):
            # If inspection started, the last waypoint in the list is HOME (0, 0)
            is_home = (i == len(self.waypoints) - 1 and self._state != "COLLECTING")
            wp_num = i + 1
            label = "HOME" if is_home else f"WP{wp_num}"

            # Sphere marker (green for normal waypoints, orange for HOME return point)
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = wp_num * 2
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wp[0]
            marker.pose.position.y = wp[1]
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            if is_home:
                # Orange sphere for home return marker
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            else:
                # Green sphere
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            marker.color.a = 0.9
            marker.lifetime = Duration(sec=0, nanosec=0)
            self.waypoint_marker_array.markers.append(marker)

            # Text label
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'waypoint_labels'
            text.id = wp_num * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = wp[0]
            text.pose.position.y = wp[1]
            text.pose.position.z = 0.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = label
            text.lifetime = Duration(sec=0, nanosec=0)
            self.waypoint_marker_array.markers.append(text)

        # 2. Add LINE_STRIP path connecting the waypoints
        if len(self.waypoints) > 0:
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns = 'waypoint_path'
            line.id = 9999
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.04  # 4cm thickness
            # Cyan/Blue color
            line.color.r = 0.0
            line.color.g = 0.8
            line.color.b = 1.0
            line.color.a = 0.7  # Semi-transparent
            line.lifetime = Duration(sec=0, nanosec=0)

            # Start from Home (0,0)
            p_home = Point()
            p_home.x = self._home_x
            p_home.y = self._home_y
            p_home.z = 0.05
            line.points.append(p_home)

            # Add all waypoints in order
            for wp in self.waypoints:
                p = Point()
                p.x = wp[0]
                p.y = wp[1]
                p.z = 0.05
                line.points.append(p)

            self.waypoint_marker_array.markers.append(line)

        self.pub_waypoint_marker.publish(self.waypoint_marker_array)

    # ------------------------------------------------------------------
    # Start inspection trigger
    # ------------------------------------------------------------------

    def _start_inspection_cb(self, msg: String):
        """Triggered by: ros2 topic pub /start_inspection std_msgs/String ..."""
        if self._state != "COLLECTING":
            self.get_logger().warn('Inspection already running!')
            return

        if len(self.waypoints) == 0:
            self.get_logger().error(
                '❌ No waypoints set! Use RViz "Publish Point" first.')
            return

        # Set state to NAVIGATING early to correct HOME marker label in RViz
        self._state = "NAVIGATING"

        # Center camera servo at start
        self.send_servo_cmd(85, 70)

        # Add home as the last waypoint
        self.waypoints.append((self._home_x, self._home_y))

        # Rebuild and update markers (so LINE_STRIP connects back to HOME)
        self._update_waypoint_markers()

        self.get_logger().warn(
            f'🚀 Starting inspection with {len(self.waypoints) - 1} '
            f'waypoints + return home!')
        for i, (x, y) in enumerate(self.waypoints):
            label = "HOME" if i == len(self.waypoints) - 1 else f"WP{i+1}"
            self.get_logger().info(f'  {label}: ({x:.2f}, {y:.2f})')

        # Start navigation in a separate thread
        threading.Thread(
            target=self._begin_navigation, daemon=True).start()

    def _begin_navigation(self):
        """Wait for Nav2 then start waypoint sequence."""
        self.get_logger().info('Waiting for Nav2 action server...')
        while not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 not ready, retrying...')
            time.sleep(1.0)

        self._state = "NAVIGATING"
        self._current_wp_idx = 0
        self.send_next_waypoint()

    # ------------------------------------------------------------------
    # Waypoint navigation
    # ------------------------------------------------------------------

    def send_next_waypoint(self):
        if self._current_wp_idx >= len(self.waypoints):
            self.get_logger().info('🎉 All waypoints completed!')
            self._state = "COMPLETED"
            return

        wx, wy = self.waypoints[self._current_wp_idx]
        is_home = (self._current_wp_idx == len(self.waypoints) - 1)
        label = "HOME" if is_home else f"WP{self._current_wp_idx + 1}"

        self.get_logger().info(
            f'🎯 [{label}] Goal {self._current_wp_idx + 1}/'
            f'{len(self.waypoints)}: ({wx:.2f}, {wy:.2f})')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wx)
        goal.pose.pose.position.y = float(wy)
        goal.pose.pose.orientation.w = 1.0

        self._state = "NAVIGATING"
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Goal {self._current_wp_idx + 1} rejected! Retrying in 5s...')
            t = threading.Thread(
                target=self._retry_after_delay, args=(5.0,), daemon=True)
            t.start()
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info(f'Goal {self._current_wp_idx + 1} accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _retry_after_delay(self, delay):
        time.sleep(delay)
        self.get_logger().info('Retrying waypoint goal...')
        self.send_next_waypoint()

    def _goal_result_callback(self, future):
        self._current_goal_handle = None
        try:
            result = future.result()
            status = result.status

            if status == 4:  # GOAL_SUCCEEDED
                is_home = (self._current_wp_idx == len(self.waypoints) - 1)

                if is_home:
                    self.get_logger().info(
                        '✅ Returned to origin successfully!')
                    if not self._report_generated:
                        self._report_generated = True
                        self.generate_report()
                    self._state = "COMPLETED"
                    # Reset camera to center on completion
                    self.send_servo_cmd(85, 70)
                    return

                # Handle waypoint reach in a separate thread so we don't block the ROS2 executor
                threading.Thread(target=self._handle_waypoint_reached, daemon=True).start()

            elif status == 5:  # GOAL_CANCELED
                self.get_logger().warn('Goal canceled (anomaly inspection).')

            else:
                self.get_logger().warn(
                    f'⚠️ Goal failed (status={status}). Retrying in 5s...')
                # Retry in a separate thread to prevent blocking
                threading.Thread(target=self._retry_failed_goal, daemon=True).start()

        except Exception as e:
            self.get_logger().error(f'Goal result error: {e}')

    def _handle_waypoint_reached(self):
        self.get_logger().info(
            f'✅ Reached WP{self._current_wp_idx + 1}! '
            f'Starting active Pan-Tilt scan...')
        self._state = "WAITING"

        # --- TẦNG 1: QUÉT SÀN (TILT_DOWN = 100) ---
        self.get_logger().info('--- TẦNG 1: QUÉT SÀN (Tìm vết nứt) ---')
        
        self.get_logger().info('  [San] Quet giua-xuong (85, 100)...')
        self.send_servo_cmd(85, 100)
        time.sleep(1.6) # Xoay 30 deg (~0.6s) + 1.0s dừng quét tĩnh
        
        self.get_logger().info('  [San] Quet trai-xuong (143, 100)...')
        self.send_servo_cmd(143, 100)
        time.sleep(2.2) # Xoay 58 deg (~1.2s) + 1.0s dừng quét tĩnh
        
        self.get_logger().info('  [San] Quet phai-xuong (23, 100)...')
        self.send_servo_cmd(23, 100)
        time.sleep(3.4) # Xoay 120 deg (~2.4s) + 1.0s dừng quét tĩnh

        # --- TẦNG 2: QUÉT KHÔNG GIAN/TƯỜNG (TILT_CENTER = 70) ---
        self.get_logger().info('--- TẦNG 2: QUÉT KHÔNG GIAN (Tim vat can/di thuong) ---')
        
        self.get_logger().info('  [Khong gian] Quet giua-ngang (85, 70)...')
        self.send_servo_cmd(85, 70)
        time.sleep(2.2) # Xoay 62 deg (~1.2s) + 1.0s dừng quét tĩnh
        
        self.get_logger().info('  [Khong gian] Quet trai-ngang (143, 70)...')
        self.send_servo_cmd(143, 70)
        time.sleep(2.2) # Xoay 58 deg (~1.2s) + 1.0s dừng quét tĩnh
        
        self.get_logger().info('  [Khong gian] Quet phai-ngang (23, 70)...')
        self.send_servo_cmd(23, 70)
        time.sleep(3.4) # Xoay 120 deg (~2.4s) + 1.0s dừng quét tĩnh

        # --- ĐƯA VỀ VỊ TRÍ MẶC ĐỊNH ---
        self.get_logger().info('Resetting camera to Center (85, 70)...')
        self.send_servo_cmd(85, 70)
        time.sleep(2.2) # Xoay 62 deg (~1.2s) + 1.0s dừng quét tĩnh

        self.get_logger().info('Active scan completed!')
        self._current_wp_idx += 1
        self.send_next_waypoint()

    def _retry_failed_goal(self):
        time.sleep(5.0)
        self.send_next_waypoint()

    # ------------------------------------------------------------------
    # Anomaly handling
    # ------------------------------------------------------------------

    def anomaly_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')
            return
        t = threading.Thread(
            target=self._handle_anomaly, args=(data,), daemon=True)
        t.start()

    def _handle_anomaly(self, data):
        with self._lock:
            if self.is_paused:
                return
            self.is_paused = True

        x = data.get('x', 0.0)
        y = data.get('y', 0.0)
        crack_count = data.get('crack_count', 0)
        anomaly_id = data.get('id', '0000')

        # Skip duplicates within 0.5m
        for old in self.detected_anomalies:
            dist = math.sqrt((x - old['x'])**2 + (y - old['y'])**2)
            if dist < 0.5:
                with self._lock:
                    self.is_paused = False
                return

        self.get_logger().warn(
            f'🚨 ANOMALY! pos=({x:.2f},{y:.2f}) cracks={crack_count}')

        # 1. Cancel current goal
        if self._current_goal_handle is not None:
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().error(f'Cancel error: {e}')

        # 2. Stop robot
        self.stop_robot()

        # 3. Mark on map (red cylinder)
        self._add_anomaly_marker(x, y, anomaly_id, crack_count)

        self.detected_anomalies.append({
            'id': anomaly_id, 'x': x, 'y': y,
            'crack_count': crack_count,
            'timestamp': data.get('timestamp', ''),
            'class': data.get('class', 'crack'),
            'confidence': data.get('confidence', 0.0),
            'image': data.get('image', ''),
        })

        # 4. Pause 3s for inspection
        time.sleep(3.0)

        # 5. Resume
        self.get_logger().info('Resuming navigation...')
        with self._lock:
            self.is_paused = False
        self.send_next_waypoint()

    def stop_robot(self):
        twist = Twist()
        self.pub_cmd_vel.publish(twist)

    # ------------------------------------------------------------------
    # Anomaly Markers (Red)
    # ------------------------------------------------------------------

    def _add_anomaly_marker(self, x, y, anomaly_id, crack_count):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'anomalies'
        marker.id = self.marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        marker.lifetime = Duration(sec=0, nanosec=0)
        self.anomaly_marker_array.markers.append(marker)
        self.marker_id += 1

        text = Marker()
        text.header.frame_id = 'map'
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = 'anomaly_labels'
        text.id = self.marker_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 0.4
        text.pose.orientation.w = 1.0
        text.scale.z = 0.15
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 0.0
        text.color.a = 1.0
        text.text = f'⚠ #{len(self.detected_anomalies)+1} ({x:.1f},{y:.1f})'
        text.lifetime = Duration(sec=0, nanosec=0)
        self.anomaly_marker_array.markers.append(text)
        self.marker_id += 1

        self.pub_anomaly_marker.publish(self.anomaly_marker_array)

    # ------------------------------------------------------------------
    # Report
    # ------------------------------------------------------------------

    def generate_report(self):
        os.makedirs(self._save_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_path = os.path.join(
            self._save_dir, f'final_report_{ts}.txt')
        with open(report_path, 'w') as f:
            f.write('=' * 50 + '\n')
            f.write('ROBOT INSPECTION FINAL REPORT\n')
            f.write(f'Generated: {ts}\n')
            f.write(f'Waypoints inspected: {len(self.waypoints) - 1}\n')
            f.write('=' * 50 + '\n\n')
            f.write(f'Waypoint route:\n')
            for i, (wx, wy) in enumerate(self.waypoints):
                label = "HOME" if i == len(self.waypoints)-1 else f"WP{i+1}"
                f.write(f'  {label}: ({wx:.2f}, {wy:.2f})\n')
            f.write(f'\nTotal anomalies: {len(self.detected_anomalies)}\n\n')
            for i, a in enumerate(self.detected_anomalies, 1):
                f.write(f'[{i}] {a["class"]} (conf={a["confidence"]:.2f})\n')
                f.write(f'    Position: ({a["x"]:.2f}, {a["y"]:.2f})\n')
                f.write(f'    Cracks:   {a["crack_count"]}\n')
                f.write(f'    Time:     {a["timestamp"]}\n')
                f.write(f'    Image:    {a["image"]}\n\n')
        self.get_logger().info(f'📄 Report saved: {report_path}')


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
