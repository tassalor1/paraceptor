#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleStatus, VehicleAttitudeSetpoint, VehicleAttitude, VehicleLocalPosition
from scipy.spatial.transform import Rotation as R
import numpy as np

# Hover thrust

class DroneControlSystem:
    def __init__(self):
        self.hover_thrust = -0.74
        self.target_position = np.array([25.0, 25.0, -2.5])
        self.K_p = 0.1
        self.first_time = True
        self.initial_distance = None

    def calculate_error(self, current_position):
        current_distance = np.linalg.norm(self.target_position - current_position)
        
        if self.first_time:
            self.first_time = False
            self.initial_distance = current_distance
            
        # Normalize the progress from 1 (start) to 0 (target)
        normalized_progress = current_distance / self.initial_distance
        return np.clip(normalized_progress, 0, 1)  # Ensure stays between 0 and 1

    def calculate_roll(self):
        return 0

    def calculate_pitch(self, current_position):
        max_pitch = np.pi/12
        min_pitch = np.pi/36
        progress = self.calculate_error(current_position)

        # Now progress goes from 1 to 0 like your original t/T went from 0 to 1
        return -(min_pitch + (max_pitch - min_pitch)*16*(progress**2)*(progress - 1)**2)

        # return -16 * max_pitch * ((progress)**2) * ((progress - 1)**2)

    def calculate_yaw(self, current_position):
        raw_error = self.target_position - current_position
        return np.arctan2(raw_error[1], raw_error[0])

    def calculate_attitude_and_thrust(self, current_position):
        theta = self.calculate_pitch(current_position)
        thrust = self.hover_thrust/np.cos(theta)
        return thrust, {
            'roll': self.calculate_roll(), 
            'pitch': theta, 
            'yaw': self.calculate_yaw(current_position)
        }
    

class OffboardAttitudeSetpoint(Node):

    def __init__(self):
        super().__init__('offboard_attitude_setpoint')
        self.timer_period = 0.02
        
        self.drone_attitude = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.rotation_frd_to_ned = np.eye(3)
        self.q_from_frd_to_ned = np.array([1.0, 0.0, 0.0, 0.0])

        self.current_position = np.array([0.0, 0.0, 0.0])

        # Vehicle state variables.
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )

        # Subscribe to vehicle status to get navigation and arming state.
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/px4_2/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        # Publisher for offboard control mode.
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/px4_2/fmu/in/offboard_control_mode',
            qos_profile)

        # Publisher for attitude setpoint.
        self.attitude_setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/px4_2/fmu/in/vehicle_attitude_setpoint',
            qos_profile)

        self.rotation_subscriber = self.create_subscription(
            VehicleAttitude,  
            '/px4_2/fmu/out/vehicle_attitude', 
            self.rotation_callback, 
            qos_profile)

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/px4_2/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile)

        self.drone_controller = DroneControlSystem()
        # self.drone_controller.set_init_target_position(np.array([25.0,5.0,-2.5]))
        
        
        self.curr_time_secs = float(self.get_clock().now().nanoseconds)/1e9


    def vehicle_status_callback(self, msg: VehicleStatus):
        """
        Update the vehicle's navigation and arming states.
        """
        self.get_logger().debug(f"Nav State: {msg.nav_state}, Arming State: {msg.arming_state}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def rotation_callback(self, msg):
        # Convert quaternion to rotation matrix
        self.q_from_frd_to_ned = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        # Create a rotation object from the quaternion
        rotation = R.from_quat(self.q_from_frd_to_ned, scalar_first=True) 
        # self.get_logger().info(f"Rotation Matrix: {rotation.as_matrix()}")
        self.rotation_frd_to_ned = rotation.as_matrix()
        # self.drone_attitude['roll'], self.drone_attitude['pitch'], self.drone_attitude['yaw'] = rotation.as_euler('xyz', degrees=True)

    # def set_new_rotation(self,drone_attitude):
    #     new_rotation = R.from_euler('xyz', [drone_attitude['roll'], drone_attitude['pitch'], drone_attitude['yaw']], degrees=False).as_matrix()
    #     new_rotation = new_rotation @ self.rotation_frd_to_ned.T
    #     return R.from_matrix(new_rotation).as_quat(scalar_first=True).astype(np.float32)


    def set_new_rotation(self,drone_attitude):
        new_rotation = R.from_euler('xyz', [drone_attitude['roll'], drone_attitude['pitch'], drone_attitude['yaw']], degrees=False).as_matrix()
        # new_rotation = self.rotation_frd_to_ned @ new_rotation
        return R.from_matrix(new_rotation).as_quat(scalar_first=True).astype(np.float32)


    def local_position_callback(self, msg):
        self.current_position = np.array([msg.x, msg.y, msg.z])
        # Log distance to target
        distance = np.linalg.norm(self.current_position - self.drone_controller.target_position)
        self.get_logger().info(f"Current pos: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}], "
                            f"Distance to target: {distance:.2f}")

    def send_attitude_setpoint(self):
        # Publish offboard mode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = True
        self.offboard_mode_pub.publish(offboard_msg)
        if (self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or 
            self.arming_state != VehicleStatus.ARMING_STATE_ARMED):
            # self.curr_time_secs = float(self.get_clock().now().nanoseconds)/1e9
            return
        # time_step = float(self.get_clock().now().nanoseconds)/1e9 - self.curr_time_secs
        # self.curr_time_secs = float(self.get_clock().now().nanoseconds)/1e9
        thrust, attitude = self.drone_controller.calculate_attitude_and_thrust(self.current_position)
        # self.get_logger().info(f"Thrust: {thrust}, pitch: {attitude['pitch']}, time step: {time_step}")
        # attitude = self.drone_controller.calculate_attitude(time_step)
        attitude_msg = VehicleAttitudeSetpoint()
        attitude_msg.timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        attitude_msg.yaw_sp_move_rate = 0.0
        attitude_msg.q_d = self.set_new_rotation(attitude)
        # attitude_msg.q_d = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        attitude_msg.thrust_body = np.array([0.0, 0.0, thrust], dtype=np.float32)
        attitude_msg.reset_integral = False
        attitude_msg.fw_control_yaw_wheel = False
        self.attitude_setpoint_pub.publish(attitude_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardAttitudeSetpoint()
    try:
        while rclpy.ok():
            node.send_attitude_setpoint()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Exiting')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()