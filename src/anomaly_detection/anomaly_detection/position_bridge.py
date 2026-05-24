#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class PositionBridge(Node):
    def __init__(self):
        super().__init__('position_bridge')

        # TF buffer — đọc từ slam_toolbox (map → base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            PointStamped, '/robot_position', 10)

        # Query TF mỗi 0.1 giây (10Hz)
        self.timer = self.create_timer(0.1, self.publish_position)

        self.get_logger().info('Position bridge started (TF-based)')

    def publish_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = 'map'
            point.point.x = tf.transform.translation.x
            point.point.y = tf.transform.translation.y
            point.point.z = 0.0
            self.pub.publish(point)

        except (LookupException, ConnectivityException, ExtrapolationException):
            pass  # Chờ TF sẵn sàng

def main(args=None):
    rclpy.init(args=args)
    node = PositionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
