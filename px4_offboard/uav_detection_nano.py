#!/usr/bin/env python3
import rclpy 
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.node import Node 

from sensor_msgs.msg import Image 
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import Float32

from cv_bridge import CvBridge 
import cv2

import torch
import numpy as np

from simple_pid import PID
import sys
import os
import imutils
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))
sys.path.append(parent_dir)
from midasModel.run import run 

from yolov5.yoloDet import YoloTRT


def process_depth_for_display(prediction, bits=1):
    if not np.isfinite(prediction).all():
        prediction = np.nan_to_num(prediction, nan=0.0, posinf=0.0, neginf=0.0)
        print("WARNING: Non-finite depth values present")
    
    depth_min = prediction.min()
    depth_max = prediction.max()
    max_val = (2**(8*bits)) - 1
    
    if depth_max - depth_min > np.finfo("float").eps:
        out = max_val * (prediction - depth_min) / (depth_max - depth_min)
    else:
        out = np.zeros(prediction.shape, dtype=prediction.dtype)
    
    out = cv2.applyColorMap(np.uint8(out), cv2.COLORMAP_INFERNO)
    # cv2.putText(out, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
    
    return out.astype("uint8" if bits == 1 else "uint16")

def create_combined_image(img, depth_img, target_width=1300, target_height=440):
    # Define the width for each half of the combined image
    half_width = target_width // 2

    # Resize both images to fit half of the target width while maintaining aspect ratio
    img_resized = cv2.resize(img, (half_width, target_height))
    depth_resized = cv2.resize(depth_img, (half_width, target_height))

    # concat horizontally
    combined_img = np.hstack((img_resized, depth_resized))

    return combined_img

class DepthDistance:
    def __init__(self, img, best_bbox = None) -> None:
        self.best_bbox = best_bbox
        self.img = img


        torch.backends.cudnn.enabled = True
        torch.backends.cudnn.benchmark = True

    def slice_img(self, img):
        height, width = img.shape[:2]
        
        if self.best_bbox is None:
            # No bounding box available, crop to center
            crop_size = 450
            x_center = width // 2
            y_center = height // 2
            
            x_min = max(0, x_center - crop_size // 2)
            y_min = max(0, y_center - crop_size // 2)
            x_max = min(width, x_center + crop_size // 2)
            y_max = min(height, y_center + crop_size // 2)
            print(" Centre of image midas depth")
        else:
            # Use the existing bounding box logic
            best_bbox1 = list(map(int, self.best_bbox))
            x_min = max(0, min(best_bbox1[0], width))
            y_min = max(0, min(best_bbox1[1], height))
            x_max = max(0, min(best_bbox1[2], width))
            y_max = max(0, min(best_bbox1[3], height))
            print(" best box midas depth")
        
        return img[y_min:y_max, x_min:x_max]

    def run_model(self):
        default_models = {
        'dpt_swin2_tiny_256': 'midasModel/weights/dpt_swin2_tiny_256.pt',
        }
        sliced_img = self.slice_img(self.img)

        # Set torch options
        torch.backends.cudnn.enabled = True
        torch.backends.cudnn.benchmark = True
        
        median_depth, prediction = run(image=sliced_img,
        model_path=default_models['dpt_swin2_tiny_256'],
        model_type='dpt_swin2_tiny_256',
        )

        return median_depth, prediction
    
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.camera_feed = self.create_publisher(
            Image,
            '/camera',
            1)

        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.listener_callback,
            qos_profile)

        self.intecpetor_trajectory = self.create_subscription(
            TrajectorySetpoint,
            '/px4_2/fmu/in/trajectory_setpoint',
            self.get_inteceptor_trajectory,
            qos_profile)

        self.inteceptor_velocity = self.create_publisher(
            TrajectorySetpoint,
            '/cv/trajectory_setpoint',
            qos_profile)

        self.model_confidence = self.create_publisher(
            Float32,
            'model_confidence',
            1)

        self.current_yaw = 0.0
        self.br = CvBridge()

        # Initialise PID controllers
        pid_x = PID(1.0, 0.1, 0.05, setpoint=0)
        pid_y = PID(1.0, 0.1, 0.05, setpoint=0)
        pid_x.output_limits = (-1, 1)
        pid_y.output_limits = (-1, 1)

        #self.cv_processor = CVProcessor(pid_x, pid_y)

        library = os.path.join(parent_dir, "yolov5/build/libmyplugins.so")
        engine = os.path.join(parent_dir, "yolov5/build/yolov5s.engine")
        self.model = YoloTRT(library=library, engine=engine, conf=0.5, yolo_ver="v5")

        # Open camera once
        gstreamer_pipeline = (
        "nvarguscamerasrc sensor-mode=1 ! " 
        "video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=60/1 ! "
        "nvvidconv ! video/x-raw, width=640, height=480, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink"
        )


        self.cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
        else:
            self.get_logger().info("Camera opened successfully")

        self.timer = self.create_timer(0.001, self.read_camera)

        # **Optional warm-up**: Run a dummy inference once at initialization to load all CUDA kernels.
        dummy_frame = np.zeros((600, 600, 3), dtype=np.uint8)
        self.model.Inference(dummy_frame)  # Warm up GPU and TensorRT once

    def read_camera(self):
        if not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return
        try:
            ros_image = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.camera_feed.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Failed to publish frame: {e}")
    

    def get_inteceptor_trajectory(self, msg):
        self.current_yaw = msg.yaw

    def listener_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

         #frame = imutils.resize(cv_image, width=400)
        start_time = time.time()
        detections, inference_time = self.model.Inference(cv_image)
        end_time = time.time()

        total_latency = end_time - start_time
        self.get_logger().info(f"Inference latency: {total_latency:.4f} seconds")


        # img, velocity_x, velocity_y, highest_conf, prediction = self.cv_processor.process_image(current_frame, self.current_yaw)

        # Process depth image for imshow
        # depth_img = process_depth_for_display(prediction)

        # Create the combined image
        # combined_img = create_combined_image(img, depth_img)

        # Display the combined image
        cv2.imshow('Detected Frame', cv_image)
        cv2.waitKey(1)

        #if velocity_x is not None and velocity_y is not None:
           # twist = TrajectorySetpoint()
            #twist.velocity[0] = velocity_x
            #twist.velocity[1] = velocity_y
            #self.inteceptor_velocity.publish(twist)

        #if highest_conf is not None:
            #print(f"Highest Confidence: {highest_conf}")
            #conf_data = Float32()
            #conf_data.data = float(highest_conf)
            #self.model_confidence.publish(conf_data)
        #else:
            #conf_data = Float32()
            #conf_data.data = 0.0
            #self.model_confidence.publish(conf_data)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
