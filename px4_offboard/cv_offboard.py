#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class PX4OffboardControl(Node):
    def __init__(self):
        super().__init__('px4_offboard_control')
        # Publishers for native PX4 offboard messages
        self.offb_ctrl_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.setpoint_count = 0
        self.commands_sent = False
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        now = self.get_clock().now()
        # Publish offboard control mode (enable position control)
        offb_mode = OffboardControlMode()
        offb_mode.timestamp = now.nanoseconds // 1000  # in microseconds
        offb_mode.position = True
        offb_mode.velocity = False
        offb_mode.acceleration = False
        offb_mode.attitude = False
        offb_mode.body_rate = False
        self.offb_ctrl_pub.publish(offb_mode)

        # Publish trajectory setpoint (target position: 0,0,5)
        traj = TrajectorySetpoint()
        traj.timestamp = now.nanoseconds // 1000
        traj.position[0] = 0.0
        traj.position[1] = 0.0
        traj.position[2] = 5.0
        # Optionally set velocities, accelerations, yaw, etc. to zero.
        self.traj_pub.publish(traj)

        self.setpoint_count += 1

        # After ~5 seconds (100 setpoints) send offboard and arm commands once.
        if self.setpoint_count >= 100 and not self.commands_sent:
            self.send_offboard_mode_command()
            self.send_arm_command()
            self.commands_sent = True

    def send_offboard_mode_command(self):
        cmd = VehicleCommand()
        cmd.timestamp = self.get_clock().now().nanoseconds // 1000
        # VEHICLE_CMD_DO_SET_MODE is usually 176; adjust as needed.
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE  
        # Param1 could be set to a mode number (e.g. 1 for offboard) – check your PX4 config.
        cmd.param1 = 1.0  
        self.cmd_pub.publish(cmd)
        self.get_logger().info("Offboard mode command sent.")

    def send_arm_command(self):
        cmd = VehicleCommand()
        cmd.timestamp = self.get_clock().now().nanoseconds // 1000
        # VEHICLE_ARM_DISARM is usually 400; adjust as needed.
        cmd.command = VehicleCommand.VEHICLE_ARM_DISARM  
        cmd.param1 = 1.0  # 1 to arm
        self.cmd_pub.publish(cmd)
        self.get_logger().info("Arm command sent.")

def main(args=None):
    rclpy.init(args=args)
    node = PX4OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
