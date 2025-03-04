# ros imports
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy, 
    QoSDurabilityPolicy
)

# Message Type Imports
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus
from geometry_msgs.msg import Vector3
from px4_msgs.msg import VehicleAttitude

# Other Imports
from scipy.spatial.transform import Rotation as R
import time
import numpy as np

class FrameHandler:
    def __init__(self, logger):
        self.logger = logger
        self.frd_to_ned_rotation = np.eye(3)
        self.last_rotation_update = 0.0
        self.rotation_update_interval = 0.5  # Update interval in seconds

    def update_frd_to_ned_rotation(self, rotation_matrix):
        current_time = time.time()
        if current_time - self.last_rotation_update >= self.rotation_update_interval:
            self.frd_to_ned_rotation = rotation_matrix
            self.last_rotation_update = current_time

    def transform_body_to_earth_frame(self, vector_input):
        return self.frd_to_ned_rotation @ vector_input
    
    def transform_earth_to_body_frame(self, vector_input):
        return self.frd_to_ned_rotation.T @ vector_input

    def get_ned_to_frd_rotation(self):
        return self.frd_to_ned_rotation.T
    
    def get_frd_to_ned_rotation(self):
        return self.frd_to_ned_rotation


class RosInterface(Node):
    def __init__(self, namespace, frame_handler=None):
        super().__init__('ros_interface')
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.rotation_matrix = np.eye(3)
        self.drone_direction_vector = np.array([1, 0, 0])
        self.current_position = np.zeros(3, dtype=np.float32)
        # Camera u, v coordinates from target tracking
        self.camera_u = 0.0  # Horizontal pixel coordinate (positive right)
        self.camera_v = 0.0  # Vertical pixel coordinate (positive down)

        if frame_handler is None:
            self.frame_handler = FrameHandler(self.get_logger())
        else:
            self.frame_handler = frame_handler

        self._setup_qos_profile()
        self._setup_publishers(namespace)
        self._setup_subscribers(namespace)

    def _setup_qos_profile(self):
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )

    def _setup_publishers(self, namespace):
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            f'/{namespace}/fmu/in/offboard_control_mode', 
            self.qos_profile
        )
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            f'/{namespace}/fmu/in/trajectory_setpoint', 
            self.qos_profile
        )

    def _setup_subscribers(self, namespace):
        self.rotation_subscription = self.create_subscription(
            VehicleAttitude, 
            f'/{namespace}/fmu/out/vehicle_attitude', 
            self._attitude_callback, 
            self.qos_profile
        )

        self.status_subscription = self.create_subscription(
            VehicleStatus, 
            f'/{namespace}/fmu/out/vehicle_status', 
            self._vehicle_status_callback, 
            self.qos_profile
        )

        # Subscribe to target tracking data (u, v coordinates)
        self.target_tracking_sub = self.create_subscription(
            Vector3, 
            '/target_tracking', 
            self._target_tracking_callback, 
            self.qos_profile
        )

    def _attitude_callback(self, msg):
        q = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        rotation = R.from_quat(q, scalar_first=True)
        self.rotation_matrix = rotation.as_matrix()
        self.drone_direction_vector = self.rotation_matrix[:, 0]
        self.frame_handler.update_frd_to_ned_rotation(self.rotation_matrix)

    def _vehicle_status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def _target_tracking_callback(self, msg):
        # Extract u and v coordinates from the message
        # We assume x component represents u (horizontal) and y represents v (vertical)
        self.camera_u = msg.x
        self.camera_v = msg.y
        self.get_logger().info(f"Camera coordinates (u, v): ({self.camera_u:.2f}, {self.camera_v:.2f})")

    def _publish_offboard_velocity_mode(self):
        # Set the drone to offboard velocity mode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        self.publisher_offboard_mode.publish(offboard_msg)

    def _publish_velocity_setpoint(self, velocity, add_hover=False):
        # Publish the setpoint message
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1e3)
        if add_hover:
            hover_velocity = np.array([0.0, 0.0, -1.0])  # Hover velocity in the Z direction
            velocity = velocity + hover_velocity
        setpoint.velocity = velocity.astype(np.float32).tolist()
        
        # Compute yaw from the x,y components (if horizontal speed is near zero, default to zero)
        vel_xy = np.array([velocity[0], velocity[1]])
        # if np.linalg.norm(vel_xy) < 1e-3:
        #     yaw = 0.0
        # else:
        #     yaw = np.arctan2(velocity[1], velocity[0])
        yaw = np.pi/2
        setpoint.yaw = float(yaw)
        setpoint.yawspeed = float('nan')
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.acceleration = [float('nan'), float('nan'), float('nan')]
        setpoint.jerk = [float('nan'), float('nan'), float('nan')]
        self.publisher_trajectory.publish(setpoint)


class CameraAlignmentController:
    def __init__(self, gain_factor=0.5):
        """
        Camera alignment controller that maps camera pixel errors to drone movement
        
        Args:
            gain_factor: Scaling factor to convert pixel errors to velocity commands
        """
        self.gain_factor = gain_factor

    def calculate_velocity_setpoint_frd(self, camera_u, camera_v):
        """
        Calculate the velocity setpoint in the Forward-Right-Down (FRD) frame
        based on camera pixel errors. The goal is to minimize u and v to 0.
        
        - If target is to the right (positive u), move right to center it
        - If target is to the left (negative u), move left to center it
        - If target is down (positive v), move down to center it
        - If target is up (negative v), move up to center it
        
        Args:
            camera_u: Horizontal pixel coordinate (positive = right)
            camera_v: Vertical pixel coordinate (positive = down)
            
        Returns:
            velocity: 3D velocity vector in FRD frame
        """
        # Negate u to map camera right to drone right
        vel_right = camera_u * self.gain_factor
        # Negate v to map camera down to drone down
        vel_down = camera_v * self.gain_factor
        # Keep forward velocity at zero for calibration
        vel_forward = 0.0
        
        velocity = 2.0*(np.array([vel_forward, vel_right, vel_down])/np.linalg.norm([vel_forward, vel_right, vel_down]))
        return velocity


class DroneController:
    def __init__(self):
        self.ros_interface = RosInterface("px4_2")
        self.frame_handler = self.ros_interface.frame_handler
        self.camera_alignment = CameraAlignmentController(gain_factor=0.5)
        self.run()

    def run(self):
        while rclpy.ok():
            # Get camera coordinates
            camera_u = self.ros_interface.camera_u
            camera_v = self.ros_interface.camera_v
            
            # Calculate the velocity setpoint in the body frame (FRD)
            velocity_frd = self.camera_alignment.calculate_velocity_setpoint_frd(camera_u, camera_v)
            
            # Transform to NED frame for publishing
            velocity_ned = self.frame_handler.transform_body_to_earth_frame(velocity_frd)
            
            # Publish velocity commands
            self.ros_interface._publish_offboard_velocity_mode()
            self.ros_interface._publish_velocity_setpoint(velocity_ned, add_hover=True)
            
            # Log information
            self.ros_interface.get_logger().info(f"Camera (u, v): ({camera_u:.2f}, {camera_v:.2f})")
            self.ros_interface.get_logger().info(f"Velocity FRD: {velocity_frd}")
            self.ros_interface.get_logger().info(f"Velocity NED: {velocity_ned}")
            
            rclpy.spin_once(self.ros_interface)
            time.sleep(0.1)  # More responsive update rate


def main(args=None):
    rclpy.init(args=args)
    try:
        controller = DroneController()
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()