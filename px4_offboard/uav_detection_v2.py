import rclpy 
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, 
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from rclpy.node import Node 

from sensor_msgs.msg import Image 
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from cv_bridge import CvBridge 
import cv2
import torch
import numpy as np

from simple_pid import PID

class CVProcessor:

    def __init__(self, pid_x, pid_y, propeller_mask_height_ratio, propeller_mask_width_ratio, vertical_offset_ratio):
        self.pid_x = pid_x
        self.pid_y = pid_y
        self.propeller_mask_height_ratio = propeller_mask_height_ratio
        self.propeller_mask_width_ratio = propeller_mask_width_ratio
        self.vertical_offset_ratio = vertical_offset_ratio
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5/weights/best.pt')

    def process_image(self, current_frame, current_yaw):
        height, width, _ = current_frame.shape
        mask = self.create_mask(height, width)
        masked_image = cv2.bitwise_and(current_frame, current_frame, mask=mask)
        results = self.model(masked_image)
        img = np.copy(results.render()[0])

        recon_centroid_x, recon_centroid_y, highest_conf = self.get_centroid(results, height, width)

        if recon_centroid_x is not None and recon_centroid_y is not None:
            error_x = recon_centroid_x - width / 2
            error_y = recon_centroid_y - height / 2

            velocity_x = self.pid_x(error_x)
            velocity_y = self.pid_y(error_y)

            self.draw_annotations(img, height, width, recon_centroid_x, recon_centroid_y)

            return img, velocity_x, velocity_y, highest_conf
        
        return img, None, None, None

    def create_mask(self, height, width):
        mask = np.ones((height, width), dtype=np.uint8) * 255
        propeller_mask_height = int(height * self.propeller_mask_height_ratio)
        propeller_mask_width = int(width * self.propeller_mask_width_ratio)
        vertical_offset = int(height * self.vertical_offset_ratio)

        mask[vertical_offset:vertical_offset + propeller_mask_height, -propeller_mask_width:] = 0
        mask[vertical_offset:vertical_offset + propeller_mask_height, :propeller_mask_width] = 0
        return mask

    def get_centroid(self, results, height, width):
        highest_conf = 7.5
        best_centroid_x, best_centroid_y = None, None
        for bbox in results.xyxy[0].cpu().numpy():
            if len(bbox) >= 6:  # Ensure there are enough values to unpack
                x_min, y_min, x_max, y_max, conf, cls = bbox
                if 0 <= x_min < width and 0 <= x_max < width and 0 <= y_min < height and 0 <= y_max < height:
                    if conf > highest_conf:
                        highest_conf = conf
                        best_centroid_x = int((x_min + x_max) / 2)
                        best_centroid_y = int((y_min + y_max) / 2)
                else:
                    print(f"Bounding Box Out of Image Bounds: {bbox}") 
        return best_centroid_x, best_centroid_y, highest_conf

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

        self.intecpetor_trajectory = self.create_subscription(
            TrajectorySetpoint,
            '/px4_2/fmu/in/trajectory_setpoint',
            self.get_inteceptor_trajectory,
            qos_profile)

        self.inteceptor_velocity = self.create_publisher(
            TrajectorySetpoint,
            'px4_2/fmu/in/trajectory_setpoint',
            qos_profile)

        self.model_confidence = self.create_publisher(
            Float32,
            'model_confidence',
            1)

        self.current_yaw = 0.0
        self.br = CvBridge()

        # Initialize PID controllers
        pid_x = PID(1.0, 0.1, 0.05, setpoint=0)
        pid_y = PID(1.0, 0.1, 0.05, setpoint=0)
        pid_x.output_limits = (-1, 1)
        pid_y.output_limits = (-1, 1)

        # Parameters for masking
        propeller_mask_height_ratio = 0.20
        propeller_mask_width_ratio = 0.20
        vertical_offset_ratio = 0.10

        self.cv_processor = CVProcessor(pid_x, pid_y, propeller_mask_height_ratio, propeller_mask_width_ratio, vertical_offset_ratio)

    def get_inteceptor_trajectory(self, msg):
        self.current_yaw = msg.yaw

    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        img, velocity_x, velocity_y, highest_conf = self.cv_processor.process_image(current_frame, self.current_yaw)

        if velocity_x is not None and velocity_y is not None:
            twist = TrajectorySetpoint()
            twist.velocity[0] = velocity_x
            twist.velocity[1] = velocity_y
            self.inteceptor_velocity.publish(twist)

        if highest_conf is not None:
            print(f"Highest Confidence: {highest_conf}")
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
