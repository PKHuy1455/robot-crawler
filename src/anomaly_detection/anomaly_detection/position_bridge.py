#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped

class PositionBridge(Node):
    def __init__(self):
        super().__init__('position_bridge')
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.callback, 10)
        self.pub = self.create_publisher(
            PointStamped,
            '/robot_position', 10)
        self.get_logger().info('Position bridge started')

    def callback(self, msg):
        point = PointStamped()
        point.header = msg.header
        point.point.x = msg.pose.pose.position.x
        point.point.y = msg.pose.pose.position.y
        point.point.z = 0.0
        self.pub.publish(point)

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
