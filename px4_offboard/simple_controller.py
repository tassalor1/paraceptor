a'''
This code uses the control system in control_system.py to calculate an IBVS trajectory for the target interception.
'''

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Message Type Imports
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleLocalPosition
from paraceptor.msg import ImageBasedVisualServo
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CameraInfo
from px4_msgs.msg import VehicleAttitude
from geometry_msgs.msg import Quaternion

from scipy.spatial.transform import Rotation as R

import time
import math
import numpy as np
from px4_offboard.control_system import ControlSystem

class InterceptorController(Node):

    def __init__(self,namespace):
        super().__init__('interceptor_controller')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=5
        )

        self.K = np.zeros((3,3)) # change for nano to calibration matrix
        self.R = np.eye(3) 
        self.target_point = Point()
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.prediction_horizon = 10
        self.control_horizon = self.prediction_horizon // 2
        self.timestep = 0.2
        self.v_max = 10

        self.control_system = ControlSystem(self.prediction_horizon, self.timestep, [0,0,0], self.v_max, self.K)

        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, f'/{namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, f'/{namespace}/fmu/in/trajectory_setpoint', qos_profile)

        # self.vehicle_local_position_subscription = self.create_subscription(VehicleLocalPosition, f'/{namespace}/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        # At the moment we can use the attitude topic, but in the actual drone we have to implement a madgwick filter
        
        self.ibvs_subscriber = self.create_subscription(ImageBasedVisualServo, '/target_tracking', self.ibvs_callback, qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, f'/{namespace}/fmu/in/command', qos_profile)
        self.status_subscription = self.create_subscription(VehicleStatus, f'/{namespace}/fmu/out/status', self.vehicle_status_callback, qos_profile)

        self.movement_timer = self.create_timer(self.timestep, self.movement_callback)

        self.previous_trajectory = None

    def ibvs_callback(self, msg):
        if(msg.bbox_perimeter):
            self.control_system.update_state([msg.bbox_perimeter, msg.deviation_x, msg.deviation_y])

    def yaw_check(self,yaw):
        # Ensure that the yaw is between -pi and pi
        if yaw > math.pi:
            return float(yaw - 2*math.pi)
        elif yaw < -math.pi:
            return float(yaw + 2*math.pi)
        else:
            return float(yaw)

    def movement_callback(self):
        
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds/1e3)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.publisher_offboard_mode.publish(offboard_msg)
        is_valid = False
        is_valid, optimal_trajectory = self.control_system.get_optimal_trajectory(self.previous_trajectory)

        if is_valid:
            setpoint = TrajectorySetpoint()
            velocity_global = optimal_trajectory[0,0:3] #np.dot(self.R,optimal_trajectory[0,0:3])
            setpoint.yawspeed = optimal_trajectory[0,3]
            setpoint.timestamp = int(self.get_clock().now().nanoseconds/1e3)
            setpoint.velocity = [float(velocity_global[0]), float(velocity_global[1]),float(velocity_global[2])]
            setpoint.yaw = float('nan')
            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
            setpoint.jerk = [float('nan'), float('nan'), float('nan')]
            self.publisher_trajectory.publish(setpoint)
            self.previous_trajectory = optimal_trajectory
        else:
            self.get_logger().info("Trajectory not calculated")
        time.sleep(0.1)

    def arm_vehicle(self):
        # Arm the vehicle if it is in OFFBOARD mode and not already armed
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            arm_command = VehicleCommand()
            arm_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            arm_command.param1 = 1.0 # 1.0 means arm
            arm_command.confirmation = 0 # no further confirmation
            arm_command.from_external = True
            self.vehicle_command_publisher_.publish(arm_command)

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state


def main(args=None):
    rclpy.init(args=args)
    interceptor_controller = InterceptorController('px4_2')
    rclpy.spin(interceptor_controller)
    interceptor_controller.destroy_node()
    rclpy.shutdown()
