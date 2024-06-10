#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleLocalPosition
from geometry_msgs.msg import Twist

class CVControl(Node):
    def __init__(self, namespace):
        super().__init__('minimal_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'/{namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        # Subscribe to altitude data
        self.altitude_sub = self.create_subscription(
            VehicleLocalPosition,
            f'/{namespace}/fmu/out/vehicle_local_position',  
            self.altitude_callback,
            qos_profile
        )

        # subscribe to twist cmds sent from camera 
        self.intecpeptor_velocity_to_target = self.create_subscription(
            TrajectorySetpoint,
            f'/{namespace}/fmu/in/trajectory_setpoint',
            self.get_cv_recon_cmd,
            qos_profile
        )
        
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, 
            f'/{namespace}/fmu/in/vehicle_command', 
            10)
        
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            f'/{namespace}/fmu/in/offboard_control_mode', 
            qos_profile)
        
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            f'/{namespace}/fmu/in/trajectory_setpoint', 
            qos_profile)
        
        self.publish_looking_twist = self.create_publisher(
            Twist,
            f'/{namespace}/fmu/in/cmd_vel',
            10
        )
        
        timer_period = 0.02 # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback) #calls the cmdloop for the specified timer_period
        self.dt = timer_period

        # for altitude scan
        self.target_height = 2.13  # 7 feet in meters
        self.current_altitude = 0.0
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.arming_timer = self.create_timer(5.0, self.arm_vehicle) # will activate function after 5 secs

        self.recon_locked_on = False

    def arm_vehicle(self):
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            arm_command = VehicleCommand()
            arm_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            arm_command.param1 = 1.0 # means arm
            arm_command.confirmation = 0 # no further confirmation
            arm_command.from_external = True
            self.vehicle_command_publisher_.publish(arm_command)
            self.get_logger().info('INTERCEPTOR Vehicle armed.')

    def vehicle_status_callback(self, msg):
        self.get_logger().info(f"CV NAV_STATUS: {msg.nav_state} - offboard status: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def altitude_callback(self, msg):
        self.current_altitude = -msg.z 
        
    def adjust_height(self):
        if self.current_altitude < self.target_height:
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = [0.0, 0.0, -self.target_height]  # Set the target height
            trajectory_msg.yaw = 0.0
            self.publisher_trajectory.publish(trajectory_msg)
            self.get_logger().info(f"Adjusting height to {self.target_height} meters")


    def get_cv_recon_cmd(self, msg):
        self.recon_locked_on = True
        self.cv_recon_x = msg.velocity[0]
        self.cv_recon_y = msg.velocity[1]
        self.cv_recon_z = msg.velocity[2]
        # self.get_logger().info(f"CV Recon Command received: x={self.cv_recon_x}, y={self.cv_recon_y}, z={self.cv_recon_z}")


    def scan_area(self):
        self.get_logger().info("Scanning for target")

        twist = Twist()
        twist.angular.z = -1.0 
        self.publish_looking_twist.publish(twist)

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.adjust_height()
            
                # if self.recon_locked_on:
                #     self.get_logger().info("Calling scan_area because recon_locked_on is False")
                self.scan_area()
                
                
def main(args=None):
    rclpy.init(args=args)
    namespace = 'px4_2'
    cv_control = CVControl(namespace=namespace)

    rclpy.spin(cv_control)

    cv_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()