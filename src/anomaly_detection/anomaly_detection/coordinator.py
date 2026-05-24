import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import json
import threading

class AnomalyCoordinator(Node):
    def __init__(self):
        super().__init__('anomaly_coordinator')

        self.is_paused = False
        self.marker_id = 0
        self._lock = threading.Lock()

        self.sub_anomaly = self.create_subscription(
            String, '/anomaly', self.anomaly_callback, 10)

        self.pub_explore_resume = self.create_publisher(
            Bool, '/explore/resume', 10)

        self.pub_cmd_vel = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.pub_marker = self.create_publisher(
            MarkerArray, '/anomaly_markers', 10)

        self.marker_array = MarkerArray()

        self.get_logger().info('Anomaly Coordinator started!')

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
        self.get_logger().info('Exploration RESUMED')

    def add_marker(self, x, y, anomaly_id, crack_count):
        # Cylinder marker đỏ
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

        # Text marker vàng
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
        text_marker.text = f'crack:{crack_count} ({x:.1f},{y:.1f})'
        text_marker.lifetime = Duration(sec=0, nanosec=0)
        self.marker_array.markers.append(text_marker)
        self.marker_id += 1

        self.pub_marker.publish(self.marker_array)

    def _handle_anomaly(self, data):
        """Chạy trong thread riêng để không block spin."""
        with self._lock:
            if self.is_paused:
                return
            self.is_paused = True

        x = data.get('x', 0.0)
        y = data.get('y', 0.0)
        crack_count = data.get('crack_count', 0)
        anomaly_id = data.get('id', '0000')

        self.get_logger().warn(
            f'⚠️  ANOMALY! pos=({x:.2f},{y:.2f}) cracks={crack_count}')

        self.pause_exploration()
        self.stop_robot()
        self.add_marker(x, y, anomaly_id, crack_count)

        # Chờ 3 giây không block spin
        import time
        time.sleep(3.0)

        self.resume_exploration()

        with self._lock:
            self.is_paused = False

    def anomaly_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')
            return

        # Chạy trong thread riêng
        t = threading.Thread(target=self._handle_anomaly, args=(data,))
        t.daemon = True
        t.start()

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
