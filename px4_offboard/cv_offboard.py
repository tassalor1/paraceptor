#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus

import time

class CVOffboardControl(Node):

    def __init__(self,namespace):
        super().__init__('cv_offboard')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'/{namespace}/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)
        
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            f'/{namespace}/fmu/in/offboard_control_mode', 
            qos_profile)

        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            f'/{namespace}/fmu/in/trajectory_setpoint', 
            qos_profile)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

    def vehicle_status_callback(self, msg):
        self.get_logger().info(f"RECON NAV_STATUS: {msg.nav_state} - offboard status: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    
    def desired_height(self):
        height_msg = TrajectorySetpoint()
        height_msg.timestamp = int(time.time() * 1e6)  # PX4 expects microseconds
        height_msg.position = [0.0, 0.0, -5.0]  # 5m up (NED frame)
        height_msg.velocity = [float('nan'), float('nan'), float('nan')]
        height_msg.acceleration = [float('nan'), float('nan'), float('nan')]
        height_msg.jerk = [float('nan'), float('nan'), float('nan')]
        height_msg.yaw = float('nan')  # No yaw control
        height_msg.yawspeed = float('nan')  # No yaw speed control
        self.publisher_trajectory.publish(height_msg)
        self.get_logger().info("desired height published")


    def cmdloop_callback(self):
        # Publish offboard control mode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.desired_height()



def main(args=None):
    rclpy.init(args=args)
    namespace='px4_2'
    offboard_control = CVOffboardControl(namespace)

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()