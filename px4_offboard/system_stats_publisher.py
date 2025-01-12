import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from jtop import jtop
import json
import struct

class SystemStatsPublisher(Node):
    def __init__(self):
        super().__init__('system_stats_publisher')
        
        self.stat_publisher = self.create_publisher(
                String,
                "mavros/system_stats", 
                10
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
            filtered_stats = [
		    cpu_avg,
		    stats.get("RAM", 0),
		    stats.get("GPU", 0), 
		    stats.get("fan", 0),
		    stats.get("Temp CPU", 0),
		    stats.get("Temp GPU", 0)
		]
            stats_json = json.dumps(filtered_stats)
            msg = String()
            msg.data = stats_json
            self.stat_publisher.publish(msg)
            
            
                        
    def destroy_node(self):
        self.jetson.close()
        self.mav.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SystemStatsPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


