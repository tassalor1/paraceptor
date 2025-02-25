#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # Publisher for the camera feed
        self.publisher = self.create_publisher(Image, '/nano_camera', 10)
        self.bridge = CvBridge()

    def read_camera(self):
        # Use GStreamer pipeline with OpenCV
        gstreamer_pipeline = (
            "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, "
            "format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink"
        )

        cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.get_logger().error("Failed to open camera")
            return

        self.get_logger().info("Camera opened successfully")

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn("Failed to capture frame")
                continue

            # Publish frame as ROS2 Image message
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f"Failed to publish frame: {e}")

            # Display the frame locally (for testing purposes)
            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        node.read_camera()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
