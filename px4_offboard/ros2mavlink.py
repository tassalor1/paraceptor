#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String
from mavros_msgs.msg import StatusText  # For sending status text messages

class Ros2Mavlink(Node):
    def __init__(self):
        super().__init__("ros2mavlink")
        
        # Create publisher to MAVROS statustext topic instead of MAVLink connection
        self.status_pub = self.create_publisher(
            StatusText,
            '/mavros/statustext/send',  # This topic exists in your list
            10
        )

        self.subscribe_rosout = self.create_subscription(
            Log, 
            '/rosout', 
            self.handle_rosout, 
            10
        )

        self.subscribe_system_stats = self.create_subscription(
            String, 
            '/mavros/system_stats', 
            self.handle_stats, 
            10
        )

        self.get_logger().info("Ros2Mavlink node started")

    def handle_rosout(self, ros_msg):
        try:
            status = StatusText()
            status.severity = 6  # INFO level (same as your original MAV_SEVERITY_INFO)
            status.text = ros_msg.msg
            self.status_pub.publish(status)
        except Exception as e:
            self.get_logger().error(f"Error sending rosout: {e}")

    def handle_stats(self, ros_msg):
        try:
            status = StatusText()
            status.severity = 6  # INFO level
            status.text = ros_msg.data
            self.status_pub.publish(status)
        except Exception as e:
            self.get_logger().error(f"Error sending stats: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Ros2Mavlink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

