#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import (OffboardControlMode, 
                          TrajectorySetpoint, 
                          VehicleLocalPosition, 
                          VehicleCommand,
                          VehicleStatus)
from geometry_msgs.msg import Point

class CVInteceptor(Node):
    def __init__(self, namespace):
        super().__init__('cv_inteceptor_control')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
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

        self.vehicle_command_publisher = self.create_publisher(
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

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        # Circle parameters
        self.radius = 6.0  # meters
        self.linear_velocity = 0.7 # meters per second
        self.altitude = 5.0  # Altitude in meters
        self.angular_velocity = self.linear_velocity / self.radius
        self.theta = 0.0  # Angle for circular motion
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.arming_timer = self.create_timer(5.0, self.arm_vehicle)  # will activate function after 5 secs

    def arm_vehicle(self):
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            arm_command = VehicleCommand()
            arm_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            arm_command.param1 = 1.0  # means arm
            arm_command.confirmation = 0  # no further confirmation
            arm_command.from_external = True
            self.vehicle_command_publisher.publish(arm_command)
            self.get_logger().info('CV INTECEPTOR Vehicle armed.')

    def vehicle_status_callback(self, msg):
        self.get_logger().info(f"CV INTECEPTOR NAV_STATUS: {msg.nav_state} - offboard status: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    
    def local_position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
      
        # Publish coordinates
        coords_msg = Point()
        coords_msg.x = self.current_x
        coords_msg.y = self.current_y
        coords_msg.z = self.current_z
        self.publisher_coords.publish(coords_msg)
       

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            x = self.radius * np.cos(self.theta)
            y = self.radius * np.sin(self.theta)
            z = -self.altitude
            
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = [x, y, z]
            trajectory_msg.yaw = self.theta  # Adjust yaw as needed, this example keeps it aligned with the circle
            self.publisher_trajectory.publish(trajectory_msg)
            
            # Update theta for the next iteration
            self.theta += self.angular_velocity * self.dt
            if self.theta >= 2 * np.pi:
                self.theta -= 2 * np.pi

def main(args=None):
    rclpy.init(args=args)
    namespace = 'px4_3'
    inteceptor_control = CVInteceptor(namespace=namespace)

    rclpy.spin(inteceptor_control)

    inteceptor_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
