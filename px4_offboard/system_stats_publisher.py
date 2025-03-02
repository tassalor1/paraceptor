#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from jtop import jtop
import json

class SystemStatsPublisher(Node):
    def __init__(self):
        super().__init__('system_stats_publisher')

        # Publish directly under PX4 namespace
        self.stat_publisher = self.create_publisher(
            String,
            "/px4_2/system_stats",
            10
        )        

        # Timer to fetch and publish stats
        self.timer = self.create_timer(2.0, self.publish_stats)

        # Initialize jtop
        self.jetson = jtop()
        self.jetson.start()
        self.get_logger().info("SystemStatsPublisher started")

    def publish_stats(self):
        if self.jetson.ok():
            # Fetch stats
            stats = self.jetson.stats
            cpu_avg = sum(stats.get(f"CPU{i}", 0) for i in range(1, 5)) / 4  # Average CPU usage
            
            # Create system stats JSON
            filtered_stats = {
                "cpu_avg": cpu_avg,
                "ram": stats.get("RAM", 0),
                "gpu": stats.get("GPU", 0),
                "fan": stats.get("fan", 0),
                "temp_cpu": stats.get("Temp CPU", 0),
                "temp_gpu": stats.get("Temp GPU", 0),
            }

            msg = String()
            msg.data = json.dumps(filtered_stats)
            self.stat_publisher.publish(msg)
            
    def destroy_node(self):
        self.jetson.close()
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


