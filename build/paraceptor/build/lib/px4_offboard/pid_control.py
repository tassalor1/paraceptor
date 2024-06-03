import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PIDControl(Node):
    def __init__(self):
        super().__init__('pid_controller')


    # PID coefficients
    self.Kp_dist = 0.4
    self.Ki_dist = 0.1
    self.Kd_dist = 0.08
    self.Kp_theta = 2
    self.Ki_theta = 0.1
    self.Kd_theta = 0.01

    # PID variables
    self.integral_dist = 0.0
    self.previous_err_dist = 0.0
    self.integral_theta = 0.0
    self.previous_err_theta = 0.0