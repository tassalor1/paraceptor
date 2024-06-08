#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand
from geometry_msgs.msg import Point


class OffboardBaseComm(Node):
    def __init__(self, namespace):
        super().__init__('OffboardBaseComm')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            f'/{namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        self.pred_pos_sub = self.create_subscription(
            Point,
            '/px4_1/fmu/out/pred_pos_5_sec',
            self.pred_pos_callback,
            qos_profile
        )

        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, f'/{namespace}/fmu/in/vehicle_command', 10)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, f'/{namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, f'/{namespace}/fmu/in/trajectory_setpoint', qos_profile)

        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.current_position = np.array([0.0, 0.0, 0.0])
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.max_velocity = 50.0

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
            self.vehicle_command_publisher_.publish(arm_command)
            self.get_logger().info('INTERCEPTOR Vehicle armed.')

    def vehicle_status_callback(self, msg):
        self.get_logger().info(f"INTERCEPTOR NAV_STATUS: {msg.nav_state} - offboard status: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def pred_pos_callback(self, msg):
        self.target_position = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f'Received Predicted Point: x={msg.x}, y={msg.y}, z={msg.z}')

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            direction_vector = self.target_position - self.current_position
            norm = np.linalg.norm(direction_vector)
            if norm > 0:
                direction_vector /= norm  # Normalize the direction vector

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = [
                self.current_position[0] + direction_vector[0] * self.timer_period * self.max_velocity,
                self.current_position[1] + direction_vector[1] * self.timer_period * self.max_velocity,
                self.current_position[2] + direction_vector[2] * self.timer_period * self.max_velocity
            ]
            self.publisher_trajectory.publish(trajectory_msg)

            # Update current position for next iteration
            self.current_position += direction_vector * self.timer_period * self.max_velocity

def main(args=None):
    rclpy.init(args=args)
    namespace = 'px4_2'
    offboard_control = OffboardBaseComm(namespace=namespace)

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()