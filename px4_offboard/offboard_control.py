#!/usr/bin/env python3
import rclpy
import numpy as np
from simple_pid import PID

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleLocalPosition
from geometry_msgs.msg import Point

class BaseStation:
    def __init__(self, init_pos_paraceptor, init_pos_enemy, v_max_paraceptor):

        # Assuming the positions are given in NED frame
        self.init_pos_paraceptor = self.ned_to_xyz(init_pos_paraceptor)
        self.init_pos_enemy = self.ned_to_xyz(init_pos_enemy)

        # Assuming the positions are given in NED frame
        self.current_pos_paraceptor = self.init_pos_paraceptor
        self.current_pos_enemy = self.init_pos_enemy

        # The maximum velocity of the paraceptor
        self.v_max_paraceptor = v_max_paraceptor # Given in m/s

        # Mode 1 is for liftoff and Mode 2 is for target mode
        self.mode = 1

        # PID gains for the paraceptor
        self.setup_PID()

        # The desired distance between the paraceptor and the enemy
        self.desired_distance_x = 0.5
        self.desired_distance_y = 0.5
        self.desired_distance_z = 0.5

    def update_paraceptor_position(self, new_pos_paraceptor):
        self.current_pos_paraceptor = self.ned_to_xyz(new_pos_paraceptor)

    def update_enemy_position(self, new_pos_enemy):
        self.current_pos_enemy = self.ned_to_xyz(new_pos_enemy)

    def ned_to_xyz(self,ned):
        return np.array([ned[1], ned[0], -ned[2]]) # x, y are the plane, z is the altitude
    
    def xyz_to_ned(self,xyz):
        return np.array([xyz[1], xyz[0], -xyz[2]])
    
    def calculate_mode(self):
        if self.current_pos_paraceptor[2] >= 0.90 * self.current_pos_enemy[2]:
            return 2
        else:
            return 2

    def setup_PID(self):
        # PID gains for the paraceptor
        self.pid_x = PID(4,0,0, setpoint=0.5) # Detonation needs to trigger at 0.5m near the target
        self.pid_y = PID(1,0,0, setpoint=0.5)
        self.pid_z = PID(1,0,0, setpoint=0.5)    

    def get_error_x(self):
        actual_distance_x = abs(self.current_pos_enemy[0] - self.current_pos_paraceptor[0])
        return self.desired_distance_x - actual_distance_x

    def get_error_y(self):
        actual_distance_y = abs(self.current_pos_enemy[1] - self.current_pos_paraceptor[1])
        return self.desired_distance_y - actual_distance_y
    
    def get_error_z(self):
        actual_distance_z = abs(self.current_pos_enemy[2] - self.current_pos_paraceptor[2])
        return self.desired_distance_z - actual_distance_z


    def calculate_velocity_vector(self):
        self.mode = self.calculate_mode()

        if self.mode == 1:
            return self.xyz_to_ned(np.array([0, 0, self.v_max_paraceptor]))
        elif self.mode == 2:
            actual_distance = np.linalg.norm(self.current_pos_enemy - self.current_pos_paraceptor)

            error_x = self.get_error_x()
            error_y = self.get_error_y()
            error_z = self.get_error_z()

            correction_x = self.pid_x(error_x)
            correction_y = self.pid_y(error_y)
            correction_z = self.pid_z(error_z)

            desired_velocity_x = (self.current_pos_enemy[0] - self.current_pos_paraceptor[0]) / actual_distance * self.v_max_paraceptor
            desired_velocity_y = (self.current_pos_enemy[1] - self.current_pos_paraceptor[1]) / actual_distance * self.v_max_paraceptor
            desired_velocity_z = (self.current_pos_enemy[2] - self.current_pos_paraceptor[2]) / actual_distance * self.v_max_paraceptor

            corrected_velocity_x = desired_velocity_x + correction_x
            corrected_velocity_y = desired_velocity_y + correction_y
            corrected_velocity_z = desired_velocity_z + correction_z

            corrected_velocity_x = np.clip(corrected_velocity_x, -self.v_max_paraceptor, self.v_max_paraceptor)
            corrected_velocity_y = np.clip(corrected_velocity_y, -self.v_max_paraceptor, self.v_max_paraceptor)
            corrected_velocity_z = np.clip(corrected_velocity_z, -self.v_max_paraceptor, self.v_max_paraceptor)

            return self.xyz_to_ned(np.array([corrected_velocity_x, corrected_velocity_y, corrected_velocity_z]))

class OffboardControl(Node):
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
        
        # subscribe to recon_coords
        self.recon_coords_sub = self.create_subscription(
            Point,
            '/px4_1/fmu/out/recon_coords',  
            self.get_recon_coords,
            qos_profile
        )
        
        # Subscribe to my own position
        # self.current_position_sub = self.create_subscription(
        #     VehicleLocalPosition,  # Assuming this is the message type
        #     f'/{namespace}/fmu/out/vehicle_local_position',
        #     self.update_current_position,
        #     10  # QoS setting
        # )        

        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, f'/{namespace}/fmu/in/vehicle_command', 10)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, f'/{namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, f'/{namespace}/fmu/in/trajectory_setpoint', qos_profile)

        timer_period = 0.02 # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback) #calls the cmdloop for the specified timer_period
        self.dt = timer_period

        self.recon_x = 0.0
        self.recon_y = 0.0
        self.recon_z = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.max_velocity = 40.0

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.arming_timer = self.create_timer(5.0, self.arm_vehicle) # will activate function after 5 secs

        self.base_station = BaseStation([0,0,0], [0,0,0], self.max_velocity)

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
        self.get_logger().info(f"INTERCEPTOR NAV_STATUS: {msg.nav_state} - offboard status: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def get_recon_coords(self, msg):
        self.recon_x = msg.x
        self.recon_y = msg.y
        self.recon_z = msg.z
        self.base_station.update_enemy_position([self.recon_x, self.recon_y, self.recon_z])

    # def update_current_position(self, msg):
    #     self.current_x = msg.x
    #     self.current_y = msg.y
    #     self.current_z = msg.z
    #     self.base_station.update_paraceptor_position([self.current_x, self.current_y, self.current_z])


    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            target_x = self.recon_x
            target_y = self.recon_y
            target_z = self.recon_z

            # Calculate the direction vector towards the target
            direction_x = target_x - self.current_x
            direction_y = target_y - self.current_y
            direction_z = target_z - self.current_z

            # Normalize the direction vector to get a unit vector
            norm = np.sqrt(direction_x**2 + direction_y**2 + direction_z**2)
            if norm > 0:
                direction_x /= norm
                direction_y /= norm
                direction_z /= norm

            # Publish TrajectorySetpoint message
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = [self.current_x + direction_x * self.dt * self.max_velocity,
                                       self.current_y + direction_y * self.dt * self.max_velocity,
                                       self.current_z + direction_z * self.dt * self.max_velocity]
            self.publisher_trajectory.publish(trajectory_msg)

            # Update current position for next iteration 
            self.current_x += direction_x * self.dt * self.max_velocity
            self.current_y += direction_y * self.dt * self.max_velocity
            self.current_z += direction_z * self.dt * self.max_velocity

    def cmdLoop_baseStation_callback(self):

        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # self.get_logger().info('Came here')

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            # Calculate the velocity vector
            velocity_vector = self.base_station.calculate_velocity_vector()
            # self.get_logger().info(f'Velocity vector: {velocity_vector}')
            # self.get_logger().info(str(self.base_station.mode))
            # Publish TrajectorySetpoint message
            trajectory_msg = TrajectorySetpoint()

            # direction = velocity_vector / np.linalg.norm(velocity_vector)

            trajectory_msg.position = [self.current_x + velocity_vector[0] * self.dt, 
                                       self.current_y + velocity_vector[1] * self.dt, 
                                       self.current_z + velocity_vector[2] * self.dt]
                        
            # trajectory_msg.velocity = [velocity_vector[0],velocity_vector[1],velocity_vector[2]]
            
            self.publisher_trajectory.publish(trajectory_msg)

            # Update current position for next iteration 
            self.current_x += velocity_vector[0] * self.dt
            self.current_y += velocity_vector[1] * self.dt
            self.current_z += velocity_vector[2] * self.dt
    
            # self.base_station.update_paraceptor_position([self.current_x, self.current_y, self.current_z])

            self.get_logger().info(f'Paraceptor velocity: {velocity_vector[0]}, {velocity_vector[1]}, {velocity_vector[2]}')

def main(args=None):
    rclpy.init(args=args)
    namespace = 'px4_2'
    offboard_control = OffboardControl(namespace=namespace)

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
