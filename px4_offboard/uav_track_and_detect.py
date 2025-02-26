#!/usr/bin/env python3
import rclpy 
from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy, 
    QoSDurabilityPolicy
)
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge 
import cv2
import torch
import numpy as np
import sys
import os
import argparse

parser = argparse.ArgumentParser(description='UAV tracking and detection')
parser.add_argument('--sim', action='store_true', help='Run in simulation mode')
args = parser.parse_args()
sim = args.sim

if not sim:
    # Get the absolute path to the 'paraceptor' directory (root)
    BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # Add yolov5 to Python path
    YOLOV5_PATH = os.path.join(BASE_DIR, "yolov5")
    sys.path.append(YOLOV5_PATH)
    # Now you can import it
    from yoloDet import YoloTRT

parser = argparse.ArgumentParser(description='UAV tracking and detection')
parser.add_argument('--sim', action='store_true', help='Run in simulation mode')
args = parser.parse_args()
sim = args.sim

print(f"Running in {'simulation' if sim else 'hardware'} mode")

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
YOLOV5_PATH = os.path.join(BASE_DIR, "yolov5")
sys.path.append(YOLOV5_PATH)
WEIGHTS_PATH = os.path.join(YOLOV5_PATH, "weights", "best.pt")
print(f"Using weights from: {WEIGHTS_PATH}")

# for nano
if not sim:
    from yoloDet import YoloTRT


class ImageProcessor:
    """
    Handles YOLO detection and tracking logic.
    - Only runs YOLO detection when needed:
      1) Tracker not initialized
      2) Tracker fails
      3) Periodic re-detection to prevent drift
      4) If YOLO's confidence is below threshold, do not initialize the tracker
    """

    def __init__(
        self, 
        propeller_mask_height_ratio, 
        propeller_mask_width_ratio, 
        vertical_offset_ratio,
        yolo_weights_path=None,
        confidence_threshold=0.5,
        re_detect_interval=30,
        simulation_mode=False
    ):
        """
        :param propeller_mask_height_ratio: float, ratio of height to mask out propeller area
        :param propeller_mask_width_ratio: float, ratio of width to mask out propeller area
        :param vertical_offset_ratio: float, ratio offset from top for the masked area
        :param yolo_weights_path: str, path to the YOLO weights
        :param confidence_threshold: float, threshold for YOLO detection confidence
        :param re_detect_interval: int, how many frames to wait before forcing a new YOLO detection
        :param simulation_mode: bool, whether to run in simulation mode
        """
        self.propeller_mask_height_ratio = propeller_mask_height_ratio
        self.propeller_mask_width_ratio = propeller_mask_width_ratio
        self.vertical_offset_ratio = vertical_offset_ratio
        self.confidence_threshold = confidence_threshold
        self.re_detect_interval = re_detect_interval
        self.simulation_mode = simulation_mode

        # Load YOLO model based on simulation mode
        if self.simulation_mode:
            print("Running in simulation mode with PyTorch model")
            # self.model = torch.hub.load(
            #     'ultralytics/yolov5', 
            #     'custom', 
            #     path=yolo_weights_path
            # )
            self.model = torch.hub.load("ultralytics/yolov5", "yolov5n")
        else:
            print("Running in hardware mode with TensorRT model")
            library = "yolov5/weights/libmyplugins.so"
            engine = "yolov5/weights/yolov5s.engine"
            self.model = YoloTRT(library=library, engine=engine, conf=0.5, yolo_ver="v5")


        # -----------------------------
        # Tracking-related members
        # -----------------------------
        self.tracker = None
        self.tracking = False  # Are we currently tracking an object?
        self.tracked_bbox = None  # (x, y, w, h)
        
        # We'll count frames since last YOLO detection
        # to decide when to forcibly re-detect
        self.frames_since_yolo = 0

    def create_mask(self, height, width):
        """
        Create a mask to avoid detecting propellers.
        """
        mask = np.ones((height, width), dtype=np.uint8) * 255
        propeller_mask_height = int(height * self.propeller_mask_height_ratio)
        propeller_mask_width = int(width * self.propeller_mask_width_ratio)
        vertical_offset = int(height * self.vertical_offset_ratio)

        mask[vertical_offset : vertical_offset + propeller_mask_height, 
             :propeller_mask_width] = 0
        mask[vertical_offset : vertical_offset + propeller_mask_height, 
             -propeller_mask_width:] = 0

        return mask

    def init_tracker(self, frame, bbox_xywh):
        """
        Initialize the tracker with (x, y, w, h).
        """
        self.tracker = cv2.TrackerCSRT_create()
        self.tracker.init(frame, tuple(bbox_xywh))
        self.tracking = True
        self.tracked_bbox = bbox_xywh
        self.frames_since_yolo = 0  # Reset every time we do a fresh detection

    def track_object(self, frame):
        """
        Update the tracker. Return new bbox (x, y, w, h) or None if failure.
        """
        if self.tracker is None:
            return None
        
        success, new_bbox = self.tracker.update(frame)
        if success:
            self.tracked_bbox = new_bbox
            return new_bbox
        else:
            # Tracking failure
            self.tracking = False
            self.tracked_bbox = None
            return None

    def run_yolo_and_init_tracker(self, frame):
        """
        Run YOLO on the masked frame, pick the best bounding box
        above a confidence threshold, initialize the tracker if found.
        Return the best bbox or None.
        """
        height, width, _ = frame.shape
        mask = self.create_mask(height, width)
        masked_image = cv2.bitwise_and(frame, frame, mask=mask)

        # Different inference call based on whether using PyTorch or TensorRT
        if self.simulation_mode:
            # PyTorch version
            results = self.model(masked_image)
            # Choose the best bounding box by confidence
            best_bbox = max(results.xyxy[0], key=lambda x: x[4]) if len(results.xyxy[0]) > 0 else None
        else:
            # TensorRT version
            results, _ = self.model.Inference(masked_image)
            # Choose the best bounding box by confidence
            best_bbox = max(results.xyxy[0], key=lambda x: x[4]) if len(results.xyxy[0]) > 0 else None

        if best_bbox is not None:
            x_min, y_min, x_max, y_max, conf, cls = best_bbox.tolist()

            # Only proceed if confidence >= threshold
            if conf >= self.confidence_threshold:
                x, y = int(x_min), int(y_min)
                w, h = int(x_max - x_min), int(y_max - y_min)

                self.init_tracker(frame, (x, y, w, h))
                return (x_min, y_min, x_max, y_max, conf, cls)
        
        # If no bbox or below threshold
        return None

    def process_image(self, current_frame):
        """
        High-level logic:
         1) If 'tracking' is True:
              - track the object
              - if tracking fails or we've passed the re-detect interval, run YOLO
         2) If 'tracking' is False:
              - run YOLO to initialize tracker
         
         Returns:
           annotated_frame, dev_x, dev_y
        """
        height, width, _ = current_frame.shape
        self.frames_since_yolo += 1

        # 1) If we're already tracking, try to update
        if self.tracking:
            # Force re-detection if we hit the re_detect_interval
            if self.frames_since_yolo >= self.re_detect_interval:
                best_bbox = self.run_yolo_and_init_tracker(current_frame)
                if best_bbox is not None:
                    # YOLO found something above threshold -> draw + dev
                    x_min, y_min, x_max, y_max, conf, cls = best_bbox
                    centroid_x = int((x_min + x_max) / 2)
                    centroid_y = int((y_min + y_max) / 2)
                else:
                    # YOLO found nothing, so we remain in "not tracking"
                    self.tracking = False
                    self.tracker = None
                    centroid_x, centroid_y = None, None
            else:
                tracked = self.track_object(current_frame)
                if tracked is not None:
                    x, y, w, h = [int(v) for v in tracked]
                    centroid_x = x + w // 2
                    centroid_y = y + h // 2

                    # Draw the rectangle
                    cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                else:
                    # Tracking failed -> run YOLO
                    best_bbox = self.run_yolo_and_init_tracker(current_frame)
                    if best_bbox is not None:
                        x_min, y_min, x_max, y_max, conf, cls = best_bbox
                        centroid_x = int((x_min + x_max) / 2)
                        centroid_y = int((y_min + y_max) / 2)
                    else:
                        self.tracking = False
                        self.tracker = None
                        centroid_x, centroid_y = None, None
        else:
            # 2) If not tracking, try YOLO
            best_bbox = self.run_yolo_and_init_tracker(current_frame)
            if best_bbox is not None:
                x_min, y_min, x_max, y_max, conf, cls = best_bbox
                centroid_x = int((x_min + x_max) / 2)
                centroid_y = int((y_min + y_max) / 2)
            else:
                centroid_x, centroid_y = None, None

        # Compute dev_x, dev_y from the center
        if centroid_x is not None and centroid_y is not None:
            dev_x = float(centroid_x - (width / 2))
            dev_y = float(centroid_y - (height / 2))

            # Draw crosshair or center line
            cv2.circle(current_frame, (centroid_x, centroid_y), 5, (0, 0, 255), -1)
            cv2.line(
                current_frame, 
                (width // 2, height // 2), 
                (centroid_x, centroid_y), 
                (0, 255, 0), 
                2
            )
        else:
            dev_x, dev_y = float('nan'), float('nan')

        return current_frame, dev_x, dev_y


class CVImagePublisher(Node):
    """
    ROS2 Node that subscribes to an Image topic, processes it via YOLO + tracking, 
    and publishes the tracking offset (Vector3).
    """

    def __init__(self, simulation_mode=False):
        super().__init__('cv_image_publisher')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )
        
        self.camera_subscriber = self.create_subscription(
            Image, 
            '/camera' if simulation_mode else '/nano_camera', 
            self.image_callback, 
            qos_profile
        )

        self.target_pred_position = self.create_publisher(
            Vector3, 
            '/target_tracking', 
            qos_profile
        )

        self.br = CvBridge()

        # Declare and read parameters
        propeller_mask_height_ratio = self.declare_parameter('propeller_mask_height_ratio', 0.20).value
        propeller_mask_width_ratio  = self.declare_parameter('propeller_mask_width_ratio',  0.20).value
        vertical_offset_ratio       = self.declare_parameter('vertical_offset_ratio',       0.10).value

        # New parameters for confidence threshold and re-detection interval
        confidence_threshold = self.declare_parameter('confidence_threshold', 0.5).value
        re_detect_interval   = self.declare_parameter('re_detect_interval', 30).value

        # You may also read the YOLO weight path from a param if desired
        yolo_weights_path = self.declare_parameter(
            'yolo_weights_path', 
            'weights/best.pt'
        ).value

        self.image_processor = ImageProcessor(
            propeller_mask_height_ratio,
            propeller_mask_width_ratio,
            vertical_offset_ratio,
            yolo_weights_path,
            confidence_threshold,
            re_detect_interval,
            simulation_mode
        )

    def image_callback(self, data):
        self.get_logger().info(f"Received image: {data.width}x{data.height} encoding: {data.encoding}")
        try:
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
            annotated_img, dev_x, dev_y = self.image_processor.process_image(current_frame)

            # Publish the tracking offsets
            msg = Vector3()
            msg.x = dev_x
            msg.y = dev_y
            msg.z = 0.0
            self.target_pred_position.publish(msg)
        
            # Display for debug
            cv2.imshow('Tracking-by-Detection', annotated_img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = CVImagePublisher(simulation_mode=sim)
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
