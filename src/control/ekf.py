import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.time import Time
from abc import abstractmethod


class EKF:
    def __init__(
        self, state_size, initial_state_covariance, initial_process_covariance
    ):
        self.state = np.zeros(state_size)
        self.P = np.diag(initial_state_covariance)  # Initialize state covariance matrix

        self.F = np.eye(state_size)  # State transition matrix
        self.Q = np.diag(initial_process_covariance)  # Process noise covariance matrix

    def predict(self, dt):
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, H, R, measurement):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        y_hat = H @ self.state
        innovation = measurement - y_hat

        self.state = self.state + K @ innovation
        self.P = (np.eye(self.P.shape[0]) - K @ H) @ self.P

    @abstractmethod
    def get_state(self):
        pass
