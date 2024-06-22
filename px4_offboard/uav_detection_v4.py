#!/usr/bin/env python3
import rclpy 
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, 
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from rclpy.node import Node 

from sensor_msgs.msg import Image 
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32

from cv_bridge import CvBridge 
import cv2
from ultralytics import YOLO 
import torch
import numpy as np

class CVProcessor:

    def __init__(self, propeller_mask_height_ratio, propeller_mask_width_ratio, vertical_offset_ratio):
        self.propeller_mask_height_ratio = propeller_mask_height_ratio
        self.propeller_mask_width_ratio = propeller_mask_width_ratio
        self.vertical_offset_ratio = vertical_offset_ratio
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/aniketh/programming/Lancelot/paraceptor_ws/src/paraceptor/yolov5/weights/best.pt')

    def create_mask(self, height, width):
        mask = np.ones((height, width), dtype=np.uint8) * 255
        propeller_mask_height = int(height * self.propeller_mask_height_ratio)
        propeller_mask_width = int(width * self.propeller_mask_width_ratio)
        vertical_offset = int(height * self.vertical_offset_ratio)

        mask[vertical_offset:vertical_offset + propeller_mask_height, -propeller_mask_width:] = 0
        mask[vertical_offset:vertical_offset + propeller_mask_height, :propeller_mask_width] = 0
        return mask

    def process_image(self, current_frame, current_yaw):
        height, width, _ = current_frame.shape
        mask = self.create_mask(height, width)
        masked_image = cv2.bitwise_and(current_frame, current_frame, mask=mask)
        results = self.model(masked_image)
        img = np.copy(results.render()[0])

        recon_centroid_x, recon_centroid_y, distance_to_target, highest_conf = self.get_centroid_and_distance(results, height, width)

        if recon_centroid_x is not None and recon_centroid_y is not None:
            target_x, target_y, target_z = self.calculate_target_position(recon_centroid_x, recon_centroid_y, distance_to_target, width, height, current_yaw)
            self.draw_annotations(img, height, width, recon_centroid_x, recon_centroid_y)
            return img, target_x, target_y, target_z, highest_conf
        return img, None, None, None, None

    def get_centroid_and_distance(self, results, height, width):
        highest_conf = 0.0
        best_centroid_x, best_centroid_y = None, None
        distance_to_target = None
        for bbox in results.xyxy[0].cpu().numpy():
            if len(bbox) >= 6:
                x_min, y_min, x_max, y_max, conf, cls = bbox
                if 0 <= x_min < width and 0 <= x_max < width and 0 <= y_min < height and 0 <= y_max < height:
                    if conf > highest_conf:
                        highest_conf = conf
                        best_centroid_x = int((x_min + x_max) / 2)
                        best_centroid_y = int((y_min + y_max) / 2)
                        W = (x_max - x_min)
                        focal_length = 882.5
                        real_world_width = 2.32
                        distance_to_target = (focal_length * real_world_width) / W
        return best_centroid_x, best_centroid_y, distance_to_target, highest_conf


    def calculate_target_position(self, recon_centroid_x, recon_centroid_y, distance_to_target, width, height, current_yaw):
        
        east = (recon_centroid_x - width / 2) / (width / 2)
        down = -(recon_centroid_y - height / 2) / (height / 2)
        north = distance_to_target

        return north,east,down

    def draw_annotations(self, img, height, width, recon_centroid_x, recon_centroid_y):
        cv2.circle(img, (recon_centroid_x, recon_centroid_y), 5, (0, 0, 255), -1)
        cv2.line(img, (int(width / 2), int(height / 2)), (recon_centroid_x, recon_centroid_y), (0, 255, 0), 2)
        cv2.putText(img, 'Recon Drone Detected', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_4)


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            1)

        self.target_pred_position = self.create_publisher(
            Point,
            '/px4_2/fmu/out/pred_pos_cv',
            qos_profile)

        self.model_confidence = self.create_publisher(
            Float32,
            'model_confidence',
            1)

        self.current_yaw = 0.0
        self.br = CvBridge()

        # Parameters for masking
        propeller_mask_height_ratio = 0.20
        propeller_mask_width_ratio = 0.20
        vertical_offset_ratio = 0.10

        self.cv_processor = CVProcessor(propeller_mask_height_ratio, propeller_mask_width_ratio, vertical_offset_ratio)

        self.last_trajectory = None

    def get_inteceptor_trajectory(self, msg):
        self.current_yaw = msg.yaw

    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        img, target_x, target_y, target_z, highest_conf = self.cv_processor.process_image(current_frame, self.current_yaw)

        print('Target position is ', target_x, target_y, target_z)
        if target_x is not None and target_y is not None and target_z is not None:
            trajectory_setpoint = Point()
            trajectory_setpoint.x = target_x
            trajectory_setpoint.y = target_y
            trajectory_setpoint.z = target_z
            self.target_pred_position.publish(trajectory_setpoint)
            self.last_trajectory = trajectory_setpoint
        else:
            self.target_pred_position.publish(self.last_trajectory)

        if highest_conf is not None:
            conf_data = Float32()
            conf_data.data = float(highest_conf)
            self.model_confidence.publish(conf_data)
        else:
            conf_data = Float32()
            conf_data.data = 0.0
            self.model_confidence.publish(conf_data)

        cv2.imshow('Detected Frame', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
