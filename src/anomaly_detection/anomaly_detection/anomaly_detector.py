#!/usr/bin/env python3
"""
anomaly_detector.py — YOLOv8n-based anomaly detection node
============================================================
Replaces the Canny edge-detection pipeline with a YOLOv8n model
(classes: crack, paint-off) trained to mAP50=0.985.

Topics
------
  Subscribed:
    /image_raw          (sensor_msgs/Image)         — camera frames
    /robot_position     (geometry_msgs/PointStamped) — from position_bridge
  Published:
    /anomaly            (std_msgs/String, JSON)      — alert → coordinator
    /anomaly_image      (sensor_msgs/Image)          — annotated frame → RViz

Parameters
----------
  model_path          str   — path to best.pt
  confidence_threshold float — YOLO confidence threshold  (default 0.45)
  imgsz               int   — inference image size px     (default 320)
  cooldown_sec        float — min seconds between alerts  (default 5.0)
  save_dir            str   — directory to save alert images

Design notes
------------
  • Inference runs in a dedicated daemon thread so the ROS spin loop is
    never blocked (Pi 4 inference ~320 ms at imgsz=320).
  • Frame queue size = 1: old frames are discarded so we always run on
    the most recent camera image.
  • /anomaly payload is JSON-compatible with the existing coordinator.py
    (adds 'class' and 'confidence' fields; 'crack_count' kept for compat).
"""

import json
import os
import queue
import threading
import time
import uuid
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ── optional YOLO import ─────────────────────────────────────────────────────

try:
    from ultralytics import YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False


# ── node ─────────────────────────────────────────────────────────────────────

class AnomalyDetector(Node):
    """YOLOv8n-powered anomaly detection node."""

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def __init__(self):
        super().__init__('anomaly_detector')

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter('model_path',
                               '/home/pi/robot_ws/models/best.pt')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('cooldown_sec', 5.0)
        self.declare_parameter('save_dir',
                               os.path.expanduser('~/robot_data/anomalies'))

        model_path   = self.get_parameter('model_path').value
        self._conf   = self.get_parameter('confidence_threshold').value
        self._imgsz  = self.get_parameter('imgsz').value
        self._cooldown = self.get_parameter('cooldown_sec').value
        self._save_dir = self.get_parameter('save_dir').value

        os.makedirs(self._save_dir, exist_ok=True)

        # ── YOLO model ────────────────────────────────────────────────
        self._model = None
        if _YOLO_AVAILABLE:
            self.get_logger().info(f'Loading YOLO model: {model_path}')
            try:
                self._model = YOLO(model_path)
                # Warmup: allocate model weights into memory now
                import numpy as np
                dummy = np.zeros((self._imgsz, self._imgsz, 3), dtype='uint8')
                self._model.predict(dummy, imgsz=self._imgsz,
                                    conf=self._conf, verbose=False)
                self.get_logger().info(
                    f'✅ YOLOv8 ready | imgsz={self._imgsz} '
                    f'| conf≥{self._conf} | classes={self._model.names}')
            except Exception as exc:
                self.get_logger().error(f'YOLO load failed: {exc}')
                self._model = None
        else:
            self.get_logger().error(
                'ultralytics not installed — detection disabled!')

        # ── state ─────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._current_x = 0.0
        self._current_y = 0.0
        self._pos_lock  = threading.Lock()
        self._last_alert_time = 0.0
        self._alert_count = 0

        # Single-slot frame queue (always process the latest frame)
        self._frame_q: queue.Queue = queue.Queue(maxsize=1)

        # ── publishers ────────────────────────────────────────────────
        self._pub_anomaly = self.create_publisher(String, '/anomaly', 10)
        self._pub_image   = self.create_publisher(Image, '/anomaly_image', 10)

        # ── subscribers ───────────────────────────────────────────────
        self.create_subscription(
            Image, '/image_raw', self._image_cb, 10)
        self.create_subscription(
            PointStamped, '/robot_position', self._position_cb, 10)

        # ── inference thread ──────────────────────────────────────────
        self._inference_thread = threading.Thread(
            target=self._inference_loop, daemon=True, name='yolo_infer')
        self._inference_thread.start()

        self.get_logger().info('🚀 AnomalyDetector (YOLOv8n) started')

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _position_cb(self, msg: PointStamped):
        """Update stored robot position (thread-safe)."""
        with self._pos_lock:
            self._current_x = msg.point.x
            self._current_y = msg.point.y

    def _image_cb(self, msg: Image):
        """Enqueue latest frame; drop old frame if queue is full."""
        try:
            # Non-blocking put: if full, discard and replace
            if self._frame_q.full():
                try:
                    self._frame_q.get_nowait()
                except queue.Empty:
                    pass
            self._frame_q.put_nowait(msg)
        except queue.Full:
            pass  # extremely unlikely with size=1, but safe

    # ------------------------------------------------------------------
    # Inference thread
    # ------------------------------------------------------------------

    def _inference_loop(self):
        """Worker thread: pull frames and run YOLO inference."""
        while rclpy.ok():
            try:
                msg = self._frame_q.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                self._process_frame(msg)
            except Exception as exc:
                self.get_logger().error(f'Inference error: {exc}')

    def _process_frame(self, msg: Image):
        """Convert ROS Image → OpenCV → YOLO → handle detections."""
        if self._model is None:
            return

        # Convert
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        # Inference
        try:
            results = self._model.predict(
                cv_img,
                imgsz=self._imgsz,
                conf=self._conf,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(f'YOLO predict error: {exc}')
            return

        # Parse detections
        result = results[0]
        detections = []
        if result.boxes is not None and len(result.boxes) > 0:
            for box in result.boxes:
                detections.append({
                    'class': self._model.names[int(box.cls[0])],
                    'confidence': float(box.conf[0]),
                    'xyxy': [round(v, 1) for v in box.xyxy[0].tolist()],
                })

        if detections:
            self._handle_detections(cv_img, result, detections)

    # ------------------------------------------------------------------
    # Detection handler
    # ------------------------------------------------------------------

    def _handle_detections(self, cv_img, result, detections: list):
        """
        Apply cooldown, save annotated image, publish /anomaly + /anomaly_image.
        """
        now = time.time()
        if now - self._last_alert_time < self._cooldown:
            return  # still in cooldown window

        self._last_alert_time = now
        self._alert_count += 1
        count = self._alert_count

        # Best detection (highest confidence)
        best = max(detections, key=lambda d: d['confidence'])
        best_class = best['class']
        best_conf  = best['confidence']

        # Robot position (snapshot)
        with self._pos_lock:
            rx, ry = self._current_x, self._current_y

        # Annotated image (YOLOv8 built-in: boxes + labels + conf)
        annotated = result.plot()

        # Overlay: position watermark
        label = (f"#{count}  {best_class} {best_conf:.2f} "
                 f"| pos ({rx:.2f},{ry:.2f})")
        cv2.putText(annotated, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                    cv2.LINE_AA)

        # Save to disk
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = f'{best_class}_{count:04d}_{ts}_x{rx:.2f}_y{ry:.2f}.jpg'
        fpath = os.path.join(self._save_dir, fname)
        cv2.imwrite(fpath, annotated)

        # ── Publish /anomaly (JSON — backward compat with coordinator) ──
        alert_data = {
            'id':          str(uuid.uuid4()),
            'timestamp':   ts,
            'x':           round(rx, 3),
            'y':           round(ry, 3),
            # legacy field kept for coordinator.py compatibility
            'crack_count': len(detections),
            # new fields
            'class':       best_class,
            'confidence':  round(best_conf, 3),
            'image':       fpath,
            'count':       count,
            'all_detections': detections,
        }
        alert_msg = String()
        alert_msg.data = json.dumps(alert_data)
        self._pub_anomaly.publish(alert_msg)

        # ── Publish annotated image for RViz ────────────────────────
        try:
            ros_img = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = 'camera_link'
            self._pub_image.publish(ros_img)
        except Exception as exc:
            self.get_logger().error(f'Image publish error: {exc}')

        self.get_logger().warn(
            f'🚨 ANOMALY #{count}: {best_class} (conf={best_conf:.3f}) '
            f'@ ({rx:.2f}, {ry:.2f}) | {len(detections)} detection(s) '
            f'| saved: {fname}'
        )


# ── entry point ──────────────────────────────────────────────────────────────

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
