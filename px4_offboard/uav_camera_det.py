import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
from ultralytics import YOLO 
import torch
import numpy as np 

model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5/weights/best.pt')


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        self.subscription = self.create_subscription(
            Image, 
            'camera', 
            self.listener_callback, 
            1)
        self.subscription  # prevent unused variable warning
        
        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')

        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")

        ''' this logic is to block out the blades as they intefer with the detection.
        it simply masks the blades'''
        height, width, _ = current_frame.shape
        mask = np.ones((height, width), dtype=np.uint8) * 255

        propeller_mask_height = int(height * 0.15)  
        propeller_mask_width = int(width * 0.17)
        vertical_offset = int(height * 0.10)

        # right propeller mask
        mask[vertical_offset:vertical_offset + propeller_mask_height, -propeller_mask_width:] = 0
        # left propeller mask 
        mask[vertical_offset:vertical_offset + propeller_mask_height, :propeller_mask_width] = 0
        # Apply  mask
        masked_image = cv2.bitwise_and(current_frame, current_frame, mask=mask)

        results = model(masked_image)
        
        # make copy so we can put centroid in box
        img = np.copy(results.render()[0]) 
        # finds centre and marks with red dot
        for bbox in results.xyxy[0].cpu().numpy():
            x_min, y_min, x_max, y_max, conf, cls = bbox
            # Calculate the centroid
            centroid_x = int((x_min + x_max) / 2)
            centroid_y = int((y_min + y_max) / 2)
            # draw the centroid
            cv2.circle(img, (centroid_x, centroid_y), 5, (0, 0, 255), -1)
           

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
