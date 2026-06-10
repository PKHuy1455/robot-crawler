import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from explore_lite_msgs.msg import ExploreStatus
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json
import threading
import os
from datetime import datetime


class AnomalyCoordinator(Node):
    def __init__(self):
        super().__init__('anomaly_coordinator')

        self.is_paused = False
        self.marker_id = 0
        self._lock = threading.Lock()
        self.detected_anomalies = []
        self._report_generated = False
        self._returning_home = False

        self.declare_parameter('save_dir', '/home/pi/robot_data/anomalies')
        self._save_dir = self.get_parameter('save_dir').value

        self.sub_anomaly = self.create_subscription(
            String, '/anomaly', self.anomaly_callback, 10)

        status_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_status = self.create_subscription(
            ExploreStatus, '/explore/status',
            self.status_callback, status_qos)

        self.pub_explore_resume = self.create_publisher(
            Bool, '/explore/resume', 10)
        self.pub_cmd_vel = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.pub_marker = self.create_publisher(
            MarkerArray, '/anomaly_markers', 10)

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self.marker_array = MarkerArray()
        self._last_resume_time = None
        self._watchdog_timer = self.create_timer(5.0, self._watchdog_callback)

        self.get_logger().info('Anomaly Coordinator started!')

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    def _watchdog_callback(self):
        if self._report_generated:
            return
        if self._last_resume_time is None:
            return
        elapsed = (self.get_clock().now() -
                   self._last_resume_time).nanoseconds / 1e9
        if elapsed > 120.0:
            self.get_logger().warn(
                'Watchdog: explore stopped after resume. Returning home...')
            self._last_resume_time = None
            self._go_home()

    # ------------------------------------------------------------------
    # Navigate về (0,0)
    # ------------------------------------------------------------------

    def _go_home(self):
        if self._returning_home or self._report_generated:
            return
        self._returning_home = True
        self.get_logger().info('Navigating back to origin (0, 0)...')

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Nav2 action server not available! Generating report anyway.')
            self._report_generated = True
            self.generate_report()
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x = 0.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Go home goal rejected!')
            self._report_generated = True
            self.generate_report()
            return
        self.get_logger().info('Going home — goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        try:
            result = future.result()
            status = result.status
            if status == 4:
                self.get_logger().info('Arrived at origin successfully!')
            else:
                self.get_logger().warn(
                    f'Go home status={status}, report anyway.')
        except Exception as e:
            self.get_logger().error(f'Goal result error: {e}')
        finally:
            if not self._report_generated:
                self._report_generated = True
                self.generate_report()

    # ------------------------------------------------------------------
    # Robot control
    # ------------------------------------------------------------------

    def stop_robot(self):
        twist = Twist()
        self.pub_cmd_vel.publish(twist)

    def pause_exploration(self):
        msg = Bool()
        msg.data = False
        self.pub_explore_resume.publish(msg)
        self.get_logger().info('Exploration PAUSED')

    def resume_exploration(self):
        msg = Bool()
        msg.data = True
        self.pub_explore_resume.publish(msg)
        self._last_resume_time = self.get_clock().now()
        self.get_logger().info('Exploration RESUMED')

    # ------------------------------------------------------------------
    # Marker
    # ------------------------------------------------------------------

    def add_marker(self, x, y, anomaly_id, crack_count):
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
        self.marker_array.markers.append(marker)
        self.marker_id += 1

        text_marker = Marker()
        text_marker.header.frame_id = 'map'
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = 'anomaly_labels'
        text_marker.id = self.marker_id
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = 0.4
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.15
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.text = f'#{len(self.detected_anomalies)} ({x:.1f},{y:.1f})'
        text_marker.lifetime = Duration(sec=0, nanosec=0)
        self.marker_array.markers.append(text_marker)
        self.marker_id += 1

        self.pub_marker.publish(self.marker_array)

    # ------------------------------------------------------------------
    # Anomaly handling
    # ------------------------------------------------------------------

    def _handle_anomaly(self, data):
        with self._lock:
            if self.is_paused:
                return
            self.is_paused = True

        x = data.get('x', 0.0)
        y = data.get('y', 0.0)
        crack_count = data.get('crack_count', 0)
        anomaly_id = data.get('id', '0000')

        self.get_logger().warn(
            f'ANOMALY! pos=({x:.2f},{y:.2f}) cracks={crack_count}')

        self.pause_exploration()
        self.stop_robot()
        self.add_marker(x, y, anomaly_id, crack_count)

        self.detected_anomalies.append({
            'id':          anomaly_id,
            'x':           x,
            'y':           y,
            'crack_count': crack_count,
            'timestamp':   data.get('timestamp', ''),
            'class':       data.get('class', 'crack'),
            'confidence':  data.get('confidence', 0.0),
            'image':       data.get('image', ''),
        })

        import time
        time.sleep(3.0)

        self.resume_exploration()

        with self._lock:
            self.is_paused = False

    def anomaly_callback(self, msg: String):
        if self._returning_home or self._report_generated:
            return
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')
            return
        t = threading.Thread(target=self._handle_anomaly, args=(data,))
        t.daemon = True
        t.start()

    # ------------------------------------------------------------------
    # Explore status
    # ------------------------------------------------------------------

    def status_callback(self, msg: ExploreStatus):
        status = msg.status
        self.get_logger().info(f'Explore status: {status}')

        if status == 'returned_to_origin':
            if self._report_generated:
                return
            self._report_generated = True
            self.get_logger().info('Returned to origin via explore_lite!')
            self.generate_report()

        elif status == 'exploration_complete':
            if self._returning_home or self._report_generated:
                return
            self.get_logger().info(
                'Exploration complete — waiting 2.0s before navigating home...')
            self._last_resume_time = None
            t = threading.Timer(2.0, self._go_home)
            t.daemon = True
            t.start()

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
            f.write('=' * 50 + '\n\n')
            f.write(
                f'Total anomalies detected: {len(self.detected_anomalies)}\n\n')
            for i, a in enumerate(self.detected_anomalies, 1):
                f.write(f'[{i}] {a["class"]} (conf={a["confidence"]:.2f})\n')
                f.write(f'    Position: ({a["x"]:.2f}, {a["y"]:.2f})\n')
                f.write(f'    Cracks:   {a["crack_count"]}\n')
                f.write(f'    Time:     {a["timestamp"]}\n')
                f.write(f'    Image:    {a["image"]}\n\n')
        self.get_logger().info(f'Report saved: {report_path}')


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
