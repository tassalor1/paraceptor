#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String
from pymavlink import mavutil

class Ros2Mavlink(Node):
    def __init__(self):
        super().__init__("ros2mavlink")
        
        # Initialize MAVLink connection
        try:
            self.mavlink_conn = mavutil.mavlink_connection(
                "/dev/ttyACM0",  
                baud=57600
            )
            self.get_logger().info("MAVLink connection established")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MAVLink connection: {e}")
            raise e

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
        # Handle Log messages from rosout
        try:
            self.mavlink_conn.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                ros_msg.msg.encode()
            )
        except Exception as e:
            self.get_logger().error(f"Error sending rosout: {e}")

    def handle_stats(self, ros_msg):
        # Handle String messages from system stats
        try:
            self.mavlink_conn.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                ros_msg.data.encode()  
            )
        except Exception as e:
            self.get_logger().error(f"Error sending stats: {e}")

    def destroy_node(self):
        try:
            self.mavlink_conn.close()
        except:
            pass
        super().destroy_node()

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

