#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleStatus,
    VehicleCommand
)

class OffboardControl(Node):
    def __init__(self, namespace='px4_1'):
        super().__init__('offboard_control')
        
        # QoS matches typical PX4 defaults (best-effort, volatile)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            f'/{namespace}/fmu/in/offboard_control_mode',
            qos_profile
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/{namespace}/fmu/in/trajectory_setpoint',
            qos_profile
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            f'/{namespace}/fmu/in/vehicle_command',
            qos_profile
        )

        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            f'/{namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        # Internal counters/states
        self.offboard_setpoint_counter = 0
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        # Timer to publish at 50Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("OffboardControl node initialized.")

    def vehicle_status_callback(self, msg):
        """Track changes to nav_state and arming_state and log them."""
        old_nav = self.nav_state
        old_arm = self.arming_state
        
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

        # Log new states whenever they change
        if old_nav != self.nav_state or old_arm != self.arming_state:
            self.get_logger().info(
                f"[vehicle_status_callback]\n"
                f"  Navigation State changed: {old_nav} -> {self.nav_state}\n"
                f"  Arming State changed: {old_arm} -> {self.arming_state}"
            )

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Helper to send a command (arm, mode change, etc.) to PX4."""
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info(
            f"[publish_vehicle_command] Command={command}, "
            f"param1={param1}, param2={param2}"
        )

    def publish_offboard_control_mode(self):
        """Publish OffboardControlMode with position enabled."""
        msg = OffboardControlMode()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position = True  # We want to control position
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_pub.publish(msg)
        self.get_logger().debug("[publish_offboard_control_mode] Offboard control mode published.")

    def publish_trajectory_setpoint(self):
        """Publish a setpoint for hovering at 2m above the ground in NED."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position = [0.0, 0.0, -2.0]  # Hover at 2m altitude (NED uses negative Z)
        msg.yaw = 0.0
        # msg.velocity could be set if you want velocity mode
        self.trajectory_setpoint_pub.publish(msg)
        self.get_logger().debug("[publish_trajectory_setpoint] Hover at 2m setpoint published.")

    def engage_offboard_mode(self):
        """Request Offboard mode from PX4."""
        # Command: VEHICLE_CMD_DO_SET_MODE, param1=1.0 (main mode), param2=6.0 (offboard submode)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("[engage_offboard_mode] Offboard mode requested.")

    def arm_vehicle(self):
        """Arm the vehicle."""
        # Command: VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0 (arm)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("[arm_vehicle] Arm requested.")

    def timer_callback(self):
        """
        Called at ~50Hz. Publishes offboard mode & setpoints continuously.
        After 50 setpoints, requests Offboard mode.
        Then, if Offboard is active, arms the vehicle (once).
        """
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        self.offboard_setpoint_counter += 1
        self.get_logger().debug(
            f"[timer_callback] Published offboard setpoint #{self.offboard_setpoint_counter}"
        )

        # After some setpoints, request Offboard
        if self.offboard_setpoint_counter == 50:
            self.get_logger().info("[timer_callback] Enough setpoints published, attempting offboard...")
            self.engage_offboard_mode()

        # If Offboard is actually engaged and we're still not armed, arm the vehicle
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and self.arming_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_vehicle()

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl(namespace='px4_1')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

