#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleLocalPosition
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

from simple_pid import PID

class GuidanceControlNavigation:
    def __init__(self):
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        # Send Trajectory update at 0.5 Hz
        self.update_interval = 0.02
        self.speed_factor = 100.0
        self.reached_threshold = 0.5 # Thresold to check if the drone reached the target point

        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.setpoint_z = 0.0

        self.setup_PID()

        self.target_set = False

    def set_target(self, x, y, z):

        # CV Assignment
        target_x = self.current_x + y
        target_y = self.current_y + x
        target_z = self.current_z - z

        # Base Station Assignment
        # target_x = x
        # target_y = y
        # target_z = z

        self.setpoint_x = target_x
        self.setpoint_y = target_y
        self.setpoint_z = target_z

        self.target_set = True

        self.pid_x.setpoint = target_x
        self.pid_y.setpoint = target_y
        self.pid_z.setpoint = target_z

    def update_current_position(self, x, y, z, yaw):
        self.current_x = x
        self.current_y = y
        self.current_z = z
        self.current_yaw = yaw

    def get_error_x(self):
        return self.setpoint_x - self.current_x
    
    def get_error_y(self):
        return self.setpoint_y - self.current_y

    def get_error_z(self):
        return self.setpoint_z - self.current_z

    def calculate_yaw_rate(self):
        target_yaw = np.arctan2(self.get_error_y(), self.get_error_z())
        yaw_error = target_yaw - self.current_yaw
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi
        yaw_rate = 1.0 * yaw_error
        return yaw_rate

    def calculate_velocity_vector(self):

        norm = np.sqrt((self.setpoint_x - self.current_x)**2 + 
                       (self.setpoint_y - self.current_y)**2 + 
                       (self.setpoint_z - self.current_z)**2)
        if norm == 0:
            norm = 1

        velocity_x = self.speed_factor*(self.setpoint_x - self.current_x)/norm
        velocity_y = self.speed_factor*(self.setpoint_y - self.current_y)/norm
        velocity_z = self.speed_factor*(self.setpoint_z - self.current_z)/norm
        # Apply PID Correction
        velocity_x += self.pid_x(self.get_error_x())
        velocity_y += self.pid_y(self.get_error_y())
        velocity_z += self.pid_z(self.get_error_z())
        # Clip the velocity
        velocity_x = np.clip(velocity_x, -self.speed_factor, self.speed_factor)
        velocity_y = np.clip(velocity_y, -self.speed_factor, self.speed_factor)
        velocity_z = np.clip(velocity_z, -self.speed_factor, self.speed_factor)

        return velocity_x, velocity_y, velocity_z

    def has_reached_setpoint(self):
        if not self.target_set:
            return True
        else:

            distance = np.sqrt((self.setpoint_x - self.current_x)**2 +
                            (self.setpoint_y - self.current_y)**2 +
                            (self.setpoint_z - self.current_z)**2)
            
            reached = (distance <= self.reached_threshold)
            if reached:
                self.target_set = False
            return reached
    
    def setup_PID(self):
        self.pid_x = PID(1.0, 0.0, 1.0, setpoint=self.setpoint_x)
        self.pid_y = PID(1.0, 0.0, 1.0, setpoint=self.setpoint_y)
        self.pid_z = PID(1.0, 0.0, 0.0, setpoint=self.setpoint_z)

    def get_targetPoint(self):
        return self.setpoint_x, self.setpoint_y, self.setpoint_z

    def get_currentPoint(self):
        return self.current_x, self.current_y, self.current_z

    def create_trajectory_message(self):
        trajectory_setpoint = TrajectorySetpoint()

        # Initialize all fields to NaN
        trajectory_setpoint.timestamp = int(Clock().now().nanoseconds / 1000)

        velocity_x, velocity_y, velocity_z = self.calculate_velocity_vector()
        
        new_x = self.current_x + velocity_x * self.update_interval
        new_y = self.current_y + velocity_y * self.update_interval
        new_z = self.current_z + velocity_z * self.update_interval

        trajectory_setpoint.position = [new_x, new_y, new_z]

        yaw_rate = self.calculate_yaw_rate()

        trajectory_setpoint.yaw = self.current_yaw + yaw_rate * self.update_interval
        trajectory_setpoint.yawspeed = yaw_rate
        
        return trajectory_setpoint

class InterceptorControl(Node):
    def __init__(self, namespace):
        super().__init__('minimal_publisher')

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

        self.cv_pred_position_sub = self.create_subscription(
            Point,
            f'/{namespace}/fmu/out/pred_pos_cv',
            self.get_cv_pred_position,
            qos_profile
        )

        self.nav_controller = GuidanceControlNavigation()
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.cmd_loop_timer = self.create_timer(0.01, self.cmdLoop_callback)        

        self.arm_vehicle()

        # self.arming_timer = self.create_timer(5.0, self.arm_vehicle)  # will activate function after 5 secs

    def get_cv_pred_position(self, msg):
        self.nav_controller.set_target(msg.x, msg.y, msg.z)
        # North is y, east is -x and up in -z
        # self.nav_controller.set_target(-0.2,4.2,-10.5)
        # self.get_logger().info(f"Target position:{msg}")

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def local_position_callback(self, msg):
        self.nav_controller.update_current_position(msg.x, msg.y, msg.z, msg.heading)

    def arm_vehicle(self):
        # Arm the vehicle if it is in OFFBOARD mode and not already armed
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            arm_command = VehicleCommand()
            arm_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            arm_command.param1 = 1.0 # 1.0 means arm
            arm_command.confirmation = 0 # no further confirmation
            arm_command.from_external = True
            self.vehicle_command_publisher_.publish(arm_command)

    def cmdLoop_callback(self):

        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED and not self.nav_controller.has_reached_setpoint():
            # Create the trajectory message
            trajectory_msg = self.nav_controller.create_trajectory_message()
            # Publish the trajectory message
            self.publisher_trajectory.publish(trajectory_msg)
            # Print the trajectory message
            self.get_logger().info(f"Trajectory Message: {trajectory_msg.position}")
        # else:
        #     self.get_logger().info(f"Reached")


def main(args=None):
    rclpy.init(args=args)
    namespace = 'px4_2'
    inteceptor_control = InterceptorControl(namespace=namespace)

    rclpy.spin(inteceptor_control)

    inteceptor_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
