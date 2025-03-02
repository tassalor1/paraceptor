#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode


class CVOffboardControl(Node):
    def __init__(self):
        super().__init__('cv_offboard')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            qos_profile
        )

        self.position_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            qos_profile
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.offboard_enabled = False
        self.timer_period = 0.5  # Increase interval so you can see debug logs more clearly
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.get_logger().info("CVOffboardControl (MavROS) started")

    def state_cb(self, msg):
        self.current_state = msg
        self.get_logger().info(
            f"[state_cb] mode: {msg.mode}, armed: {msg.armed}, connected: {msg.connected}"
        )

    def desired_height(self):
        setpoint = PositionTarget()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        setpoint.position.x = 0.0
        setpoint.position.y = 0.0
        setpoint.position.z = -1.0
        setpoint.yaw = 0.0
        self.position_pub.publish(setpoint)
        self.get_logger().info("Published desired height setpoint")

    def cmdloop_callback(self):
        if not self.offboard_enabled:
            # Publish a few setpoints first so FCU will accept OFFBOARD
            self.get_logger().info("Publishing initial setpoints...")
            for _ in range(10):
                self.desired_height()
                time.sleep(0.05)

            # Uncomment below if you actually want to arm and switch to OFFBOARD:
            # self.arm_and_offboard()
            self.offboard_enabled = True
        else:
            self.desired_height()

    def arm_and_offboard(self):
        # Wait for services
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Arming service not available, waiting...")

        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("SetMode service not available, waiting...")

        if not self.current_state.armed:
            arm_req = CommandBool.Request()
            arm_req.value = True
            resp_arming = self.arming_client.call(arm_req)
            if resp_arming and resp_arming.success:
                self.get_logger().info("Vehicle armed")
            else:
                self.get_logger().error("Arming failed")

        if self.current_state.mode != "OFFBOARD":
            offb_req = SetMode.Request()
            offb_req.custom_mode = "OFFBOARD"
            resp_offb = self.set_mode_client.call(offb_req)
            if resp_offb and resp_offb.mode_sent:
                self.get_logger().info("OFFBOARD enabled")
            else:
                self.get_logger().error("Offboard mode request failed")


def main(args=None):
    rclpy.init(args=args)
    node = CVOffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

