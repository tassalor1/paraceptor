import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
from ultralytics import YOLO 

# load the YOLOv8 model
model = YOLO('yolov8m.pt')


class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'camera', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function
    """
    self.get_logger().info('Receiving video frame')
 
    # convert ros Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
    image = current_frame
    # Object Detection
    results = model.predict(image, classes=[0, 2])
    img = results[0].plot()

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