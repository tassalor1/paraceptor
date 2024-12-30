#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, 
                       QoSHistoryPolicy, QoSDurabilityPolicy)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, 
                          VehicleStatus, VehicleCommand, VehicleLocalPosition)


class TestFlight(Node):
    def __init__(self, namespace):
        super().__init__('test_flight')
        
        # Parameters for the test flight
        self.radius = 10.0  # Circle radius in meters
        self.linear_velocity = 2.0  # m/s
        self.altitude = 5.0  # Altitude in meters
        self.test_duration = 30.0  # Total test time in seconds

        self.angular_velocity = self.linear_velocity / self.radius
        self.theta = 0.0  # Initial angle for the circular motion
        self.flight_timer = 0.0  # Timer to track total flight time

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Subscriptions and Publishers
        self.create_subscription(
            VehicleStatus, f'/{namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile
        )
        self.create_subscription(
            VehicleLocalPosition, f'/{namespace}/fmu/out/vehicle_local_position',
            self.local_position_callback, qos_profile
        )
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, f'/{namespace}/fmu/in/vehicle_command', 10
        )
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, f'/{namespace}/fmu/in/offboard_control_mode', qos_profile
        )
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, f'/{namespace}/fmu/in/trajectory_setpoint', qos_profile
        )

        # Timers
        self.dt = 0.02  # Control loop period (50Hz)
        self.timer = self.create_timer(self.dt, self.cmdloop_callback)
        self.create_timer(5.0, self.arm_vehicle)  # Arm the vehicle after 5 seconds

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Store initial position
        self.starting_position = None

    def arm_vehicle(self):
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            arm_command = VehicleCommand()
            arm_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            arm_command.param1 = 1.0  # Arm the drone
            self.vehicle_command_publisher_.publish(arm_command)
            self.get_logger().info('Vehicle armed.')

    def hold_vehicle(self):
        hold_command = VehicleCommand()
        hold_command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        hold_command.param1 = 1.0  # Main mode: Auto
        hold_command.param2 = 3.0  # Sub mode: Hold
        hold_command.target_system = 1
        hold_command.target_component = 1
        hold_command.source_system = 1
        hold_command.source_component = 1
        hold_command.from_external = True
        self.vehicle_command_publisher_.publish(hold_command)
        self.get_logger().info("Hold command sent. Vehicle holding position.")


    def land_vehicle(self):
        land_command = VehicleCommand()
        land_command.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        land_command.param1 = 0.0  # Reserved (set to 0.0)
        land_command.param2 = 0.0  # Reserved (set to 0.0)
        land_command.param3 = 0.0  # Reserved (set to 0.0)
        land_command.param4 = 0.0  # Desired yaw angle (optional, 0.0 to keep current yaw)
        land_command.param5 = float('nan')  # Latitude (optional, NaN for current position)
        land_command.param6 = float('nan')  # Longitude (optional, NaN for current position)
        land_command.param7 = float('nan')  # Altitude (optional, NaN for current altitude)
        land_command.target_system = 1
        land_command.target_component = 1
        land_command.source_system = 1
        land_command.source_component = 1
        land_command.from_external = True
        self.vehicle_command_publisher_.publish(land_command)
        self.get_logger().info("Land command sent. Vehicle descending to land.")


    def disarm_vehicle(self):
        disarm_command = VehicleCommand()
        disarm_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        disarm_command.param1 = 0.0  # Disarm the drone
        self.vehicle_command_publisher_.publish(disarm_command)
        self.get_logger().info('Vehicle disarmed.')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def local_position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

        # Store the starting position on the first callback
        if self.starting_position is None:
            self.starting_position = (self.current_x, self.current_y, self.current_z)
            self.get_logger().info(f"Starting position: x={self.current_x}, y={self.current_y}, z={self.current_z}")

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        self.publisher_offboard_mode.publish(offboard_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            if self.starting_position is None:
                return  # Wait until the starting position is received

            x = self.radius * np.cos(self.theta)
            y = self.radius * np.sin(self.theta)
            z = -self.altitude

            # Ensure altitude stays within safe bounds
            z = max(z, -10.0)  # No lower than -10 meters
            
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = [x, y, z]
            trajectory_msg.yaw = self.theta  # Keep yaw aligned with the circle
            self.publisher_trajectory.publish(trajectory_msg)

            self.get_logger().info(f"Flying to: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={self.theta:.2f}")

            # Update angle and flight timer
            self.theta += self.angular_velocity * self.dt
            self.flight_timer += self.dt

            # Reset theta after a full circle
            if self.theta >= 2 * np.pi:
                self.theta -= 2 * np.pi

            # If flight time is over, return to start
            if self.flight_timer >= self.test_duration:
                self.return_to_start()
                self.get_logger().info("Returning to start position...")
                
                self.flight_timer = 0  # Reset timer after returning to start

    def return_to_start(self):
        # Move the drone back to its starting position
        if self.starting_position:
            x_start, y_start, z_start = self.starting_position
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = [x_start, y_start, z_start]
            trajectory_msg.yaw = 0.0  # Align yaw to the start direction
            self.publisher_trajectory.publish(trajectory_msg)

            # self.hold_vehicle()
            # # self.land_vehicle()
            # # # Disarm the drone after returning to the start
            # # self.disarm_vehicle()

def main(args=None):
    rclpy.init(args=args)
    namespace = 'px4_3'
    test_flight = TestFlight(namespace=namespace)
    rclpy.spin(test_flight)
    test_flight.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
