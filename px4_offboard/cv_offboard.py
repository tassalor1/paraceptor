#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy, 
    QoSDurabilityPolicy
)
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
        super().__init__('hardware_test')

        # Use Foxy- & Humble-compatible QoS constants
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
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

        # Publishers
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

        #self.publisher_trajectory = self.create_publisher(
            #TrajectorySetpoint, 
            #f'/{namespace}/fmu/in/trajectory_setpoint', 
            #qos_profile
        #)

        self.publisher_coords = self.create_publisher(
            Point, 
            f'/{namespace}/fmu/out/recon_coords', 
            qos_profile
        )

        # Timing
        self.dt = 3 # 20ms
        self.timer = self.create_timer(self.dt, self.cmdloop_callback)

        # Circle parameters
        self.radius = 10.0         # meters
        self.linear_velocity = 1.0 # m/s
        self.altitude = 5.0        # meters
        self.angular_velocity = self.linear_velocity / self.radius
        self.theta = 0.0

        # PX4 states
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Arm vehicle timer
        self.arming_timer = self.create_timer(5.0, self.arm_vehicle)

    def arm_vehicle(self):
        """Attempt to arm vehicle if OFFBOARD mode is set."""
        self.get_logger().info(f"Checking if vehicle can arm... NAV_STATE={self.nav_state}, ARMING_STATE={self.arming_state}")
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            arm_command = VehicleCommand()
            arm_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            arm_command.param1 = 1.0  # Arm
            arm_command.confirmation = 0
            arm_command.from_external = True
            self.vehicle_command_publisher_.publish(arm_command)
            self.get_logger().info('RECON Vehicle armed command sent.')

    def vehicle_status_callback(self, msg):
        self.get_logger().info(
            f"Vehicle Status Update: NAV_STATE={msg.nav_state}, ARMING_STATE={msg.arming_state}"
        )
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def local_position_callback(self, msg):
        # Capture local position
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

        # Log local position
        self.get_logger().info(
            f"Local Position: x={self.current_x:.3f}, y={self.current_y:.3f}, z={self.current_z:.3f}"
        )

        # Publish coords
        coords_msg = Point()
        coords_msg.x = self.current_x
        coords_msg.y = self.current_y
        coords_msg.z = self.current_z
        self.publisher_coords.publish(coords_msg)

    def cmdloop_callback(self):
        """Periodically publish offboard control and trajectory setpoints."""
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)
        self.get_logger().info("Offboard control mode message published.")

        # Only publish trajectory if OFFBOARD + ARMED
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and 
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            #x = self.radius * np.cos(self.theta)
            #y = self.radius * np.sin(self.theta)
            #z = -self.altitude  # PX4 uses negative altitude for "up"

            #trajectory_msg = TrajectorySetpoint()
            #trajectory_msg.position = [x, y, z]
            #trajectory_msg.yaw = self.theta  # Keep yaw in sync with circle angle
            #self.publisher_trajectory.publish(trajectory_msg)

            #self.get_logger().info(
                #f"Publishing trajectory: x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={self.theta:.3f}"
            #)
            self.get_logger().info(
                f" we would be Publishing trajectory now"
            )

            # Increment angle
            self.theta += self.angular_velocity * self.dt
            if self.theta >= 2.0 * np.pi:
                self.theta -= 2.0 * np.pi
        else:
            self.get_logger().info(
                f"Offboard not set or vehicle not armed: "
                f"NAV_STATE={self.nav_state}, ARMING_STATE={self.arming_state}"
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
