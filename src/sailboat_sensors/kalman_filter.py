import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from .airmar_node import AirMar

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter')

        # Declare parameters for timer period
        self.declare_parameter('timer_period', 0.5)
        self.timer_period = self.get_parameter('timer_period').value

        # Publishers
        self.publisher_filtered_gps = self.create_publisher(NavSatFix, '/filtered_gps', 10)

        # Timer
        self.timer = self.create_timer(self.timer_period, self.kalman_callback)

        # Initialize AirMar Node
        self.airmar = AirMar()

        # Kalman Filter Initialization
        self.dt = 0.1

        # State vector [x, y, theta, v, omega]
        self.x = np.zeros((5, 1))

        # State covariance matrix (identity matrix)
        self.P = np.eye(5)

        # Control input matrix
        self.B = np.zeros((5, 2))

        # Process noise covariance Q
        self.Q = np.diag([0.1, 0.1, 0.01, 0.1, 0.01])

        # Measurement noise covariance R
        self.R = np.diag([0.5, 0.5])

        # Measurement matrix H
        self.H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ])

    def kalman_callback(self):
        # Read measurements from AirMar
        lat = self.airmar.getLat()
        lon = self.airmar.getLong()
        heading = self.airmar.getHeading()
        velocity = self.airmar.getVelocity()
        angular_velocity = self.airmar.getAngularVelocity()

        # Update state transition matrix A
        self.x[2, 0] = heading
        self.x[3, 0] = velocity
        self.x[4, 0] = angular_velocity

        self.A = np.array([
            [1, 0, -self.x[3, 0] * np.sin(self.x[2, 0]) * self.dt, np.cos(self.x[2, 0]) * self.dt, 0],
            [0, 1, self.x[3, 0] * np.cos(self.x[2, 0]) * self.dt, np.sin(self.x[2, 0]) * self.dt, 0],
            [0, 0, 1, 0, self.dt],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

        # Control input vector
        u = np.zeros((2, 1))

        # Prediction Step
        self.x, self.P = self.predict(self.x, self.P, self.A, self.B, u, self.Q)

        # Measurement Vector
        z = np.array([[lat], [lon]])

        # Update Step
        self.x, self.P = self.update(self.x, self.P, z, self.H, self.R)

        # Publish filtered GPS position
        nav_sat_msg = NavSatFix()
        nav_sat_msg.latitude = self.x[0, 0]
        nav_sat_msg.longitude = self.x[1, 0]
        nav_sat_msg.position_covariance_type = 0
        self.publisher_filtered_gps.publish(nav_sat_msg)

        self.get_logger().info('Publishing Filtered GPS Data: ' + 'Lat: ' + str(nav_sat_msg.latitude) + ' Long: ' + str(nav_sat_msg.longitude))

    def predict(self, x, P, A, B, u, Q):
        # Predict the next state
        x = A @ x + B @ u
        # Update state covariance matrix
        P = A @ P @ A.T + Q
        return x, P

    def update(self, x, P, z, H, R):
        # Kalman gain
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        # Update state with measurement z
        y = z - H @ x  # Measurement residual
        x = x + K @ y
        # Update covariance matrix
        P = (np.eye(len(x)) - K @ H) @ P
        return x, P

def main(args=None):
    rclpy.init(args=args)

    kalman_filter = KalmanFilter()

    rclpy.spin(kalman_filter)

    # Destroy the node explicitly
    kalman_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()