#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        
        # Create publishers
        self.position_pub = self.create_publisher(
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

        # Initialize variables
        self.current_state = State()
        self.pose = PoseStamped()
        self.pose.pose.position.z = 2.0  # Target height of 2 meters

        # Create timer for publishing setpoints
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz
        self.setpoint_counter = 0

        self.get_logger().info('Offboard control node initialized')

    def state_callback(self, msg: State):
        """Callback for vehicle state updates"""
        # Log state changes
        if (msg.mode != self.current_state.mode or 
            msg.armed != self.current_state.armed or 
            msg.connected != self.current_state.connected):
            
            self.get_logger().info(
                f'State Update:\n'
                f'  Connected: {msg.connected}\n'
                f'  Armed: {msg.armed}\n'
                f'  Mode: {msg.mode}'
            )
        
        self.current_state = msg

    async def arm(self):
        """Send an arm command to the vehicle"""
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Arming service not available, waiting...')

        request = CommandBool.Request()
        request.value = True

        future = self.arming_client.call_async(request)
        self.get_logger().info('Arm command sent')
        return await future

    async def set_mode(self, mode: str):
        """Send a mode change command to the vehicle"""
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Set mode service not available, waiting...')

        request = SetMode.Request()
        request.custom_mode = mode

        future = self.set_mode_client.call_async(request)
        self.get_logger().info(f'Mode change requested: {mode}')
        return await future

    async def timer_callback(self):
        """Timer callback for publishing setpoints and handling mode changes"""
        # Update timestamp
        self.pose.header = Header()
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.frame_id = "base_link"

        # Publish position setpoint
        self.position_pub.publish(self.pose)
        self.setpoint_counter += 1

        # Log every 100 setpoints
        if self.setpoint_counter % 100 == 0:
            self.get_logger().info(f'Published setpoint #{self.setpoint_counter}')

        # After 100 setpoints, try to switch to offboard mode and arm
        if self.setpoint_counter == 100:
            self.get_logger().info('Attempting transition to offboard mode...')
            
            # Switch to offboard mode
            if self.current_state.mode != "OFFBOARD":
                if await self.set_mode("OFFBOARD"):
                    self.get_logger().info('Offboard mode enabled')
                else:
                    self.get_logger().warn('Failed to set OFFBOARD mode')
                    return

            # Arm the vehicle
            if not self.current_state.armed:
                if await self.arm():
                    self.get_logger().info('Vehicle armed')
                else:
                    self.get_logger().warn('Failed to arm')

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        offboard_control.get_logger().info('Node stopped cleanly')
    finally:
        offboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

