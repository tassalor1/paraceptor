#!/usr/bin/env python3
import rclpy 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from paraceptor.msg import ImageBasedVisualServo
from cv_bridge import CvBridge 
import cv2
import torch
import numpy as np

class ImageProcessor:
    def __init__(self, propeller_mask_height_ratio, propeller_mask_width_ratio, vertical_offset_ratio):
        self.propeller_mask_height_ratio = propeller_mask_height_ratio
        self.propeller_mask_width_ratio = propeller_mask_width_ratio
        self.vertical_offset_ratio = vertical_offset_ratio
        # self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/aniketh/programming/Lancelot/interception_ws/src/visual_guidance/visual_guidance/weights/best.pt')
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/aniketh/programming/Templar/intercept_ws/src/visual_guidance/visual_guidance/weights/best.pt')

    def create_mask(self, height, width):
        mask = np.ones((height, width), dtype=np.uint8) * 255
        propeller_mask_height = int(height * self.propeller_mask_height_ratio)
        propeller_mask_width = int(width * self.propeller_mask_width_ratio)
        vertical_offset = int(height * self.vertical_offset_ratio)

        mask[vertical_offset:vertical_offset + propeller_mask_height, :propeller_mask_width] = 0
        mask[vertical_offset:vertical_offset + propeller_mask_height, -propeller_mask_width:] = 0
        return mask

    def process_image(self, current_frame):
        height, width, _ = current_frame.shape
        mask = self.create_mask(height, width)
        masked_image = cv2.bitwise_and(current_frame, current_frame, mask=mask)
        results = self.model(masked_image)
        img = results.render()[0].copy()

        best_bbox = max(results.xyxy[0], key=lambda x: x[4]) if len(results.xyxy[0]) > 0 else None
        if best_bbox is not None:
            x_min, y_min, x_max, y_max, conf, cls = best_bbox
            centroid_x, centroid_y = int((x_min + x_max) / 2), int((y_min + y_max) / 2)
            
            dev_x = (centroid_x - width / 2) # Right is positive
            dev_y = (centroid_y - height / 2) / (height / 2) # Down is positive
            bbox_perimeter = 2*((x_max - x_min) + (y_max - y_min))

            cv2.circle(img, (centroid_x, centroid_y), 5, (0, 0, 255), -1)
            cv2.line(img, (int(width / 2), int(height / 2)), (centroid_x, centroid_y), (0, 255, 0), 2)

            return img, float(dev_x), float(dev_y), float(bbox_perimeter)
        return img, float('nan'), float('nan'), float('nan')

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.camera_subscriber = self.create_subscription(Image, '/camera', self.image_callback, qos_profile)
        self.target_pred_position = self.create_publisher(ImageBasedVisualServo, '/target_tracking', qos_profile)
        self.br = CvBridge()

        self.image_processor = ImageProcessor(
            self.declare_parameter('propeller_mask_height_ratio', 0.20).value,
            self.declare_parameter('propeller_mask_width_ratio', 0.20).value,
            self.declare_parameter('vertical_offset_ratio', 0.10).value
        )

    def image_callback(self, data):
        try:
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
            img, norm_dev_x, norm_dev_y, norm_bbox_perimeter = self.image_processor.process_image(current_frame)

            msg = ImageBasedVisualServo()
            msg.deviation_x = norm_dev_x
            msg.deviation_y = norm_dev_y
            msg.bbox_perimeter = norm_bbox_perimeter
            self.target_pred_position.publish(msg)
        
            cv2.imshow('Detected Frame', img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args = None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()