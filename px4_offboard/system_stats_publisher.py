import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from jtop import jtop
import json
import pymavlink.mavutil as mavutil
import struct

class SystemStatsMavSender(Node):
    def __init__(self):
        super().__init__('system_stats_publisher')
        
        # Initialize telemetry connection
        # Replace '/dev/ttyUSB0' with your telemetry port
        self.mav = mavutil.mavlink_connection(
            '/dev/ttyUSB0',
            baud=57600
        )
        
        # Timer to fetch and publish stats
        self.timer = self.create_timer(2.0, self.publish_stats)
        
        # Initialize jtop
        self.jetson = jtop()
        self.jetson.start()
        self.get_logger().info("SystemStatsMavSender started")

    def publish_stats(self):
        if self.jetson.ok():
            # Fetch stats
            stats = self.jetson.stats
            cpu_avg = (
                stats.get("CPU1", 0) +
                stats.get("CPU2", 0) +
                stats.get("CPU3", 0) +
                stats.get("CPU4", 0)
            ) / 4
            
            # Pack stats into MAVLink message
            # Using STATUSTEXT message type for custom data
            filtered_stats = {
                'cpu_avg': cpu_avg,
                'ram': stats.get("RAM", 0),
                'gpu': stats.get("GPU", 0),
                'fan': stats.get("fan", 0),
                'temp_cpu': stats.get("Temp CPU", 0),
                'temp_gpu': stats.get("Temp GPU", 0),
            }
            
            # Convert to JSON and send as STATUSTEXT
            stats_json = json.dumps(filtered_stats)
            self.mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                stats_json.encode()
            )
            
            self.get_logger().info(f"Sent MAVLink message: {stats_json}")

    def destroy_node(self):
        self.jetson.close()
        self.mav.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SystemStatsMavSender()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


