#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand,
                          VehicleLocalPosition)
from geometry_msgs.msg import Point

class InteceptorControl(Node):
    def __init__(self, namespace):
        super().__init__('inteceptor')

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
        
        # subscribe to recon_coords
        self.recon_coords_sub = self.create_subscription(
            Point,
            '/px4_1/fmu/out/pred_pos_5_sec',  
            self.get_recon_coords,
            qos_profile
        )

        # subscribe to twist cmds sent from camera 
        self.cv_cmd_sub = self.create_subscription(
            TrajectorySetpoint,
            '/cv/trajectory_setpoint',
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
        

        timer_period = 0.02 # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback) #calls the cmdloop for the specified timer_period
        self.dt = timer_period

        self.recon_x = 0.0
        self.recon_y = 0.0
        self.recon_z = 0.0

        self.cv_recon_x = 0.0
        self.cv_recon_y = 0.0
        self.cv_recon_z = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.arming_timer = self.create_timer(5.0, self.arm_vehicle) # will activate function after 5 secs

        self.recon_locked_on = False

    def arm_vehicle(self):
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            arm_command = VehicleCommand()
            arm_command.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            arm_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            arm_command.param1 = 1.0  # Arm
            arm_command.target_system = 1
            arm_command.target_component = 1
            arm_command.source_system = 1
            arm_command.source_component = 1
            arm_command.from_external = True
            self.vehicle_command_publisher_.publish(arm_command)
            self.get_logger().info('INTERCEPTOR Vehicle armed.')


    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state


    def local_position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
      

    def get_recon_coords(self, msg):
        self.get_logger().info("Received predicted position message")
        self.recon_x = msg.x
        self.recon_y = msg.y
        self.recon_z = msg.z
        self.get_logger().info(f"Received Predicted Position: x={self.recon_x}, y={self.recon_y}, z={self.recon_z}")


    def get_cv_recon_cmd(self, msg):
        self.recon_locked_on = True
        self.cv_recon_x = msg.velocity[0]
        self.cv_recon_y = msg.velocity[1]
        self.cv_recon_z = msg.velocity[2]
    
    # def follow_recon_from_home_station(self):
    #     # calc the direction vector towards the target
    #     direction_x = self.recon_x - self.current_x
    #     direction_y = self.recon_y - self.current_y
    #     direction_z = self.recon_z - self.current_z

    #     # norm the direction vector to get a unit vector
    #     norm = np.sqrt(direction_x**2 + direction_y**2 + direction_z**2)
    #     if norm > 0:
    #         direction_x /= norm
    #         direction_y /= norm
    #         direction_z /= norm

    #     # correct yaw so it faces forward
    #     yaw = np.arctan2(direction_y, direction_x)

    #     # Publish TrajectorySetpoint message
    #     trajectory_msg = TrajectorySetpoint()
    #     trajectory_msg.position = [self.current_x + direction_x * self.dt * 10,
    #                                 self.current_y + direction_y * self.dt * 10,
    #                                 self.current_z + direction_z * self.dt * 10]
    #     trajectory_msg.yaw = yaw
    #     self.publisher_trajectory.publish(trajectory_msg)
        

    #     speed_factor = 100
    #     # Update current position for next iteration 
    #     self.current_x += direction_x * self.dt * speed_factor
    #     self.current_y += direction_y * self.dt * speed_factor
    #     self.current_z += direction_z * self.dt * speed_factor

    def follow_recon_from_home_station(self):
        # Since we are using predicted positions, set them directly as the target
        target_x = self.recon_x
        target_y = self.recon_y
        target_z = self.recon_z

        # Calculate the direction vector towards the target
        direction_x = target_x - self.current_x
        direction_y = target_y - self.current_y
        direction_z = target_z - self.current_z

        self.get_logger().info(f"Current Position: x={self.current_x}, y={self.current_y}, z={self.current_z}")
        self.get_logger().info(f"Predicted Target Position: x={target_x}, y={target_y}, z={target_z}")
        self.get_logger().info(f"Direction Vector: x={direction_x}, y={direction_y}, z={direction_z}")

        # Calculate yaw to face the target
        yaw = np.arctan2(direction_y, direction_x)

        # Create and publish TrajectorySetpoint message with predicted target position
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        trajectory_msg.position = [target_x, target_y, target_z]
        trajectory_msg.yaw = yaw
        self.publisher_trajectory.publish(trajectory_msg)


    
    def follow_recon_from_cv(self):
        # self.get_logger().info("following recon with CV cmds")
        # calc the direction vector towards the target
        direction_x = self.cv_recon_x - self.current_x
        direction_y = self.cv_recon_y - self.current_y
        direction_z = self.cv_recon_z - self.current_z


        # correct yaw so it faces forward
        yaw = np.arctan2(direction_y, direction_x)

        # Publish TrajectorySetpoint message
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [self.current_x + direction_x * self.dt * 10,
                                    self.current_y + direction_y * self.dt * 10,
                                    self.current_z + direction_z * self.dt * 10]
        trajectory_msg.yaw = yaw
        self.publisher_trajectory.publish(trajectory_msg)
        

        speed_factor = 100
        # Update current position for next iteration 
        self.current_x += direction_x * self.dt * speed_factor
        self.current_y += direction_y * self.dt * speed_factor
        self.current_z += direction_z * self.dt * speed_factor

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.publisher_offboard_mode.publish(offboard_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Vehicle is in OFFBOARD mode and ARMED.")
            self.follow_recon_from_home_station()
        
            # if self.recon_locked_on:
            #     self.follow_recon_from_cv()
            # else:
            #     self.follow_recon_from_home_station()
            
def main(args=None):
    rclpy.init(args=args)
    namespace = 'px4_2'
    inteceptor_control = InteceptorControl(namespace=namespace)

    rclpy.spin(inteceptor_control)

    inteceptor_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
