#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymavlink import mavutil
import json


class Ros2Mavlink(Node):
    def __init__(self):
        super().__init__("ros2mavros")
        self.mavlink_conn = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600)

        self.ros_msg = self.create_subscription(
            String,
            'mavros/system_stats',
            self.convert_ros_2_mavlink,
            10
        )

        self.get_logger().info("Ros 2 mavros Node started")

    def convert_ros_2_mavlink(self, ros_msg):
        try:
            # Parse JSON string into a Python list
            telemetry_data = json.loads(ros_msg.data)
            self.get_logger().info(f'Parsed telemetry data: {telemetry_data}')

            # Ensure telemetry data is valid
            if not isinstance(telemetry_data, (list, tuple)):
                raise TypeError("Telemetry data must be a list or tuple")

            # Format telemetry data into a single string
            formatted_data = ",".join(map(str, telemetry_data))
            formatted_data = formatted_data.encode("ascii", "ignore").decode("ascii")
            formatted_data = formatted_data[:50]  # Truncate to 50 characters
            self.get_logger().info(f"Formatted telemetry data: {formatted_data}")

            # Send all stats as a single statustext message
            self.send_statustext_message(formatted_data)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def send_statustext_message(self, data):
        try:
            # Send telemetry as a statustext message
            self.get_logger().info(f"Sending statustext: {data}")
            mav_msg = self.mavlink_conn.mav.statustext_encode(
                severity=6,  # Info severity
                text=data
            )
            mav_msg.pack(self.mavlink_conn.mav)
            self.mavlink_conn.mav.send(mav_msg)
            self.get_logger().info(f"Sent MAVLink statustext: {data}")
        except Exception as e:
            self.get_logger().error(f"Error sending statustext message: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Ros2Mavlink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

