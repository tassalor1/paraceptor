import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.optimize import minimize

import time
import numpy as np

import matplotlib.pyplot as plt
from threading import Thread

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class DroneKalmanFilter:
    def __init__(self, dt=0.02, R_std=0.1, Q_std=0.01):
        self.dt = dt
        self.kf = KalmanFilter(dim_x=6, dim_z=3)

        # State transition matrix (models the system dynamics)
        self.kf.F = np.array([[1, 0, 0, dt, 0, 0],
                              [0, 1, 0, 0, dt, 0],
                              [0, 0, 1, 0, 0, dt],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])

        # Measurement function (maps state to measurement space)
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0]])

        # Initial state covariance
        self.kf.P *= 100.

        # Measurement noise
        self.kf.R = np.eye(3) * R_std

        # Process noise for a 6D state [x, y, z, vx, vy, vz]
        q = Q_discrete_white_noise(dim=3, dt=dt, var=Q_std)
        self.kf.Q = np.zeros((6, 6))
        self.kf.Q[:3, :3] = q
        self.kf.Q[3:, 3:] = q

        # Initial state (assuming starting at origin with zero velocity)
        self.kf.x = np.array([0., 0., 0., 0., 0., 0.])

    def update(self, measurement):
        self.kf.predict()
        self.kf.update(measurement)
        return self.kf.x[:3]  # Return only the position part

    def predict_future(self, steps):
        future_state = self.kf.x.copy()
        for _ in range(steps):
            future_state = self.kf.F @ future_state
        return future_state[:3]  # Return only the position part

def cost_function(params, measurements, dt, future_steps):
    R_std, Q_std = params
    drone_kf = DroneKalmanFilter(dt=dt, R_std=R_std, Q_std=Q_std)
    total_error = 0
    for i in range(len(measurements) - 1):
        updated_state = drone_kf.update(measurements[i])
        predicted_state = drone_kf.predict_future(future_steps)
        actual_state = measurements[min(i + future_steps, len(measurements) - 1)]
        total_error += np.sum((predicted_state - actual_state) ** 2)
    return total_error

class DroneSubscriber(Node):

    def __init__(self):
        super().__init__('base_station')
        self.timer_threshold = 0.2
        self.start_time = time.time()
        self.dt = 0.05
        self.future_seconds = 10.0  # Predicting 5 seconds into the future
        self.future_steps = int(self.future_seconds / self.dt)  # This will be 250
        self.measurements = []

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.enemy_pos_sub = self.create_subscription(
            Point,
            '/px4_1/fmu/out/recon_coords',
            self.listener_callback,
            qos_profile
        )

        self.predicted_ppos_pub = self.create_publisher(Point, '/px4_1/fmu/out/pred_pos_5_sec', qos_profile)

        self.kalman_filter = DroneKalmanFilter(dt=self.dt, R_std=0.1, Q_std=0.01)

    def listener_callback(self, msg):
        current_time = time.time()
        if current_time - self.start_time > self.timer_threshold:
            measurement = np.array([msg.x, msg.y, msg.z])
            self.measurements.append(measurement)
            if len(self.measurements) > 10:  # Start processing after collecting enough measurements
                self.optimize_kalman_filter()
            updated_state = self.kalman_filter.update(measurement)
            # self.get_logger().info(f'Received Point: x={msg.x}, y={msg.y}, z={msg.z}')
            
            # Next step prediction
            next_step_prediction = self.kalman_filter.predict_future(1)
            # self.get_logger().info(f'Next Step Prediction: {next_step_prediction}')

            # Publish the predicted position 5 seconds into the future
            future_point = Point()
            # 5 seconds into the future prediction
            future_prediction = self.kalman_filter.predict_future(self.future_steps)
            future_point.x, future_point.y, future_point.z = future_prediction
            self.predicted_ppos_pub.publish(future_point)
            # self.get_logger().info(f'Prediction {self.future_seconds} seconds into the future: {future_prediction}')

            self.start_time = current_time

    def optimize_kalman_filter(self):
        initial_params = [0.1, 0.01]
        result = minimize(cost_function, initial_params, args=(self.measurements, self.dt, self.future_steps), bounds=[(0.001, 10), (0.001, 10)])
        optimized_R_std, optimized_Q_std = result.x
        self.kalman_filter = DroneKalmanFilter(dt=self.dt, R_std=optimized_R_std, Q_std=optimized_Q_std)
        # self.get_logger().info(f'Optimized R_std: {optimized_R_std}, Optimized Q_std: {optimized_Q_std}')

def main(args=None):
    rclpy.init(args=args)
    drone_subscriber = DroneSubscriber()
    rclpy.spin(drone_subscriber)
    drone_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()