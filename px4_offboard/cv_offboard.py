#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import sys

class CVOffboardControl(Node):
    def __init__(self):
        super().__init__('cv_offboard')
        
        # Create publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Create subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        # Create service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services synchronously
        self.ensure_service_availability()

        # Initialize variables
        self.current_state = State()
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 2.0  # Target height

        # Create timer for publishing setpoints at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_request = self.get_clock().now()

        self.get_logger().info('Offboard control node started')

    def ensure_service_availability(self):
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Arming service...')
        self.get_logger().info('Arming service is now available.')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Set mode service...')
        self.get_logger().info('Set mode service is now available.')

    def state_callback(self, msg):
        self.current_state = msg

    def timer_callback(self):
        # Update timestamp and publish position setpoint
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.frame_id = "map"
        self.local_pos_pub.publish(self.pose)

        # Every 5 seconds, attempt mode change or arming
        if (self.get_clock().now() - self.last_request).nanoseconds / 1e9 > 5.0:
            if self.current_state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")
            elif not self.current_state.armed:
                self.arm()
            self.last_request = self.get_clock().now()

    def arm(self):
        self.get_logger().info("Arming...")
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def set_mode(self, mode):
        self.get_logger().info(f"Setting mode to {mode}...")
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def move_to(self, x, y, z):
        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = z
        self.get_logger().info(f'Moving to position: x={x}, y={y}, z={z}')

def main():
    rclpy.init()
    offboard = CVOffboardControl()
    
    try:
        rclpy.spin(offboard)
    except KeyboardInterrupt:
        offboard.get_logger().info('Stopping offboard control...')
    finally:
        offboard.set_mode("AUTO.LOITER")
        offboard.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
