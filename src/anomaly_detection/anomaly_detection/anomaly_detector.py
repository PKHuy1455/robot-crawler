import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
import uuid
from datetime import datetime

class AnomalyDetector(Node):
    def __init__(self):
        super().__init__('anomaly_detector')
        
        # Declare parameters
        self.declare_parameter('canny_threshold1', 50)
        self.declare_parameter('canny_threshold2', 150)
        self.declare_parameter('min_crack_area', 100)
        self.declare_parameter('blur_kernel', 5)
        self.declare_parameter('cooldown_sec', 3.0)
        
        self.bridge = CvBridge()
        
        # Current robot position
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Cooldown management
        self.last_detection_time = 0
        
        # Ensure output directory exists
        self.output_dir = os.path.expanduser('~/robot_data/anomalies')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Publishers and Subscribers
        self.sub_image = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
            
        self.sub_position = self.create_subscription(
            PointStamped,
            '/robot_position',
            self.position_callback,
            10)
            
        self.pub_anomaly = self.create_publisher(String, '/anomaly', 10)
        
        self.get_logger().info('Anomaly Detector Node initialized')

    def position_callback(self, msg: PointStamped):
        self.current_x = msg.point.x
        self.current_y = msg.point.y

    def image_callback(self, msg: Image):
        # Check cooldown (compare in nanoseconds)
        current_time = self.get_clock().now().nanoseconds
        cooldown_ns = int(self.get_parameter('cooldown_sec').value * 1e9)
        if current_time - self.last_detection_time < cooldown_ns:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Parameters
        thresh1 = self.get_parameter('canny_threshold1').value
        thresh2 = self.get_parameter('canny_threshold2').value
        min_area = self.get_parameter('min_crack_area').value
        blur_k = self.get_parameter('blur_kernel').value

        # Ensure blur kernel is odd
        if blur_k % 2 == 0:
            blur_k += 1

        # Pipeline
        # 1. Grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 2. Gaussian Blur
        blurred = cv2.GaussianBlur(gray, (blur_k, blur_k), 0)
        
        # 3. Canny
        edges = cv2.Canny(blurred, thresh1, thresh2)
        
        # 4. Morphology (Closing to connect small gaps)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        morph = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # 5. findContours
        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by area
        valid_contours = [c for c in contours if cv2.contourArea(c) > min_area]
        
        if len(valid_contours) > 0:
            self.last_detection_time = current_time
            self.handle_detection(cv_image, valid_contours)

    def handle_detection(self, original_image, contours):
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        x_str = f"{self.current_x:.2f}"
        y_str = f"{self.current_y:.2f}"
        
        # Draw contours for saved image (optional but helpful for visualization)
        annotated_image = original_image.copy()
        cv2.drawContours(annotated_image, contours, -1, (0, 0, 255), 2)
        
        # Save image
        filename = f"{timestamp_str}_{x_str}_{y_str}.jpg"
        filepath = os.path.join(self.output_dir, filename)
        cv2.imwrite(filepath, annotated_image)
        
        # Publish JSON alert
        alert_data = {
            "id": str(uuid.uuid4()),
            "timestamp": timestamp_str,
            "x": self.current_x,
            "y": self.current_y,
            "crack_count": len(contours)
        }
        
        alert_msg = String()
        alert_msg.data = json.dumps(alert_data)
        self.pub_anomaly.publish(alert_msg)
        
        self.get_logger().info(f"Anomaly detected! Saved to {filename}. Alert published.")

def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

