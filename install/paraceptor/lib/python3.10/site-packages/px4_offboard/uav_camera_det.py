import rclpy 
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, 
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from rclpy.node import Node 

from sensor_msgs.msg import Image 
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge 
import cv2
from ultralytics import YOLO 
import torch
import numpy as np 

model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5/weights/best.pt')

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
        
        self.intecpetor_trajectory = self.create_subscription(
            TrajectorySetpoint, 
            '/px4_2/fmu/in/trajectory_setpoint', 
            self.get_inteceptor_trajectory,
            qos_profile)
        
        self.inteceptor_velocity = self.create_publisher(
           TrajectorySetpoint,
           'px4_2/fmu/in/trajectory_setpoint',
           qos_profile)

        self.current_yaw = 0.0

        self.cvfont = cv2.FONT_HERSHEY_SIMPLEX 

        self.br = CvBridge()


    def get_inteceptor_trajectory(self, msg):
        ''' gets inteceptor trajectory from topic '''
        self.current_yaw = msg.yaw

    def inteceptor_movement_to_recon_centre(self,
                                            recon_centroid_x, recon_centroid_y,
                                            direction_point_x, direction_point_y
                                            ):
    
        dx = recon_centroid_x - direction_point_x
        dy = recon_centroid_y - direction_point_y

        magnitude = (dx**2 + dy**2)**0.5
        direction_x = dx / magnitude
        direction_y = dy / magnitude

        k_p = 0.1  # Proportional gain
        velocity_x = k_p * direction_x
        velocity_y = k_p * direction_y

        twist = TrajectorySetpoint()
        twist.velocity[0] = velocity_x
        twist.velocity[1] = velocity_y

        self.inteceptor_velocity.publish(twist)
       
    def listener_callback(self, data):

        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")

        ''' this logic is to block out the blades as they intefer with the detection.
        it simply masks the blades'''
        height, width, _ = current_frame.shape
        mask = np.ones((height, width), dtype=np.uint8) * 255

        # size of the masks
        propeller_mask_height = int(height * 0.20)  
        propeller_mask_width = int(width * 0.20)
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

        # Calculate the direction point relative to the center of the image
        direction_length = 20  
        direction_point_x = int(width / 2 + direction_length * np.cos(self.current_yaw))
        direction_point_y = int(height / 2 - direction_length * np.sin(self.current_yaw))

        # draw direction point
        cv2.circle(img, (direction_point_x, direction_point_y), 5, (255, 0, 0), -1)  

        recon_centroid_x, recon_centroid_y = None, None

        # finds centre and marks with red dot
        for bbox in results.xyxy[0].cpu().numpy():
            x_min, y_min, x_max, y_max, conf, cls = bbox
            # if boundig box is in within image
            if 0 <= x_min < width and 0 <= x_max < width and 0 <= y_min < height and 0 <= y_max < height:
                
              # Calculate the recon drones centroid
              r_centroid_x = int((x_min + x_max) / 2)
              r_centroid_y = int((y_min + y_max) / 2)

              recon_centroid_x = r_centroid_x
              recon_centroid_y = r_centroid_y
              # draw the centroid
              cv2.circle(img, (r_centroid_x, r_centroid_y), 5, (0, 0, 255), -1)

        # if drone centroid then cretae line
        if recon_centroid_x is not None and recon_centroid_y is not None:
           
            # publish twist msg to topic
            self.inteceptor_movement_to_recon_centre(
                                                    recon_centroid_x, recon_centroid_y,
                                                    direction_point_x, direction_point_y
                                                    )
            cv2.line(img, 
                    (direction_point_x, direction_point_y), 
                    (recon_centroid_x, recon_centroid_y), 
                    (0, 255, 0), 2)

            cv2.putText(img,  
                'Recon Drone Detected',  
                (50, 50),  
                self.cvfont, 1,  
                (0, 255, 255),  
                2,  
                cv2.LINE_4) 
           
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
