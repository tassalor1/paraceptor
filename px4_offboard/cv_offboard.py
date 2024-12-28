#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleStatus,
    VehicleCommand,
    VehicleLocalPosition
)
from geometry_msgs.msg import Point

class ReconControl(Node):
    def __init__(self, namespace):
        super().__init__('recon_control')
        
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'/{namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )
        
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f'/{namespace}/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile
        )

        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, 
            f'/{namespace}/fmu/in/vehicle_command', 
            10
        )
        
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            f'/{namespace}/fmu/in/offboard_control_mode', 
            qos_profile
        )
        
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            f'/{namespace}/fmu/in/trajectory_setpoint', 
            qos_profile
        )
        
        self.publisher_coords = self.create_publisher(
            Point, 
            f'/{namespace}/fmu/out/recon_coords', 
            qos_profile
        )

        # 20 ms loop
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)
        
        # Smaller circle for real drone testing
        self.radius = 3.0
        self.altitude = 3.0
        self.linear_velocity = 2.0
        self.angular_velocity = self.linear_velocity / self.radius
        self.theta = 0.0
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Arm after 5s if we have OFFBOARD
        self.arming_timer = self.create_timer(5.0, self.arm_vehicle)

    def arm_vehicle(self):
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            arm_cmd = VehicleCommand()
            arm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            arm_cmd.param1 = 1.0  # arm
            arm_cmd.from_external = True
            self.vehicle_command_publisher_.publish(arm_cmd)
            self.get_logger().info('RECON Vehicle armed.')

    def vehicle_status_callback(self, msg):
        self.get_logger().info(f"RECON NAV_STATUS: {msg.nav_state}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    
    def local_position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

        coords_msg = Point()
        coords_msg.x = self.current_x
        coords_msg.y = self.current_y
        coords_msg.z = self.current_z
        self.publisher_coords.publish(coords_msg)

    def cmdloop_callback(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        self.publisher_offboard_mode.publish(offboard_msg)

        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
                self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            x = self.radius * np.cos(self.theta)
            y = self.radius * np.sin(self.theta)
            z = -self.altitude

            traj_msg = TrajectorySetpoint()
            traj_msg.position = [x, y, z]
            traj_msg.yaw = self.theta
            self.publisher_trajectory.publish(traj_msg)
            self.get_logger().info(f"Publishing trajectory: x={x}, y={y}, z={z}, yaw={self.theta}")

            self.theta += self.angular_velocity * self.timer_period
            if self.theta >= 2.0 * np.pi:
                self.theta -= 2.0 * np.pi
        else:
            self.get_logger().info(
                f"Offboard not set or vehicle not armed: nav_state={self.nav_state}, arming={self.arming_state}"
            )

def main(args=None):
    rclpy.init(args=args)
    namespace = 'px4_1'
    recon_control = ReconControl(namespace=namespace)
    rclpy.spin(recon_control)
    recon_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
