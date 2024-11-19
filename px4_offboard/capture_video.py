#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 

from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
import os

class CaptureCvVideo(Node):
    def __init__(self):
        super().__init__("cv_video_capture")

        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.capture_cv,
            10
        )
        self.br = CvBridge()
        self.save_dir = "/paraceptor/cv_capture_data"
        self.save_every_n_frame = 5
        self.frame_count = 0
        self.image_counter = 0

    def capture_cv(self, data):
        self.frame_count += 1

        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
       
        if self.frame_count % self.save_every_n_frame == 0:
            self.image_counter += 1
            filename = os.path.join(self.save_dir, f"img_{self.image_counter}.png")

            try:
                cv2.imwrite(filename, current_frame)
            except Exception as e:
                self.get_logger().error(f"Failed to save image {filename}: {e}")

        cv2.imshow("camera feed", current_frame)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = CaptureCvVideo()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()