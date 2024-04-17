## takes in accelerometer IMU and combines to output state to ROS code

import numpy as np
from scipy.linalg import block_diag

class ExtendedKalmanFilter:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        self.state = np.array(initial_state)
        self.covariance = np.array(initial_covariance)
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def predict(self, u, F, dt):
        # State prediction
        self.state = np.dot(F, self.state) + u * dt
        # Covariance prediction
        Ft = np.transpose(F)
        self.covariance = np.dot(F, np.dot(self.covariance, Ft)) + self.process_noise

    def update(self, z, H):
        # Measurement update
        Z = np.dot(H, self.state)
        Y = z - Z
        Ht = np.transpose(H)
        S = np.dot(H, np.dot(self.covariance, Ht)) + self.measurement_noise
        K = np.dot(self.covariance, np.dot(Ht, np.linalg.inv(S)))
        self.state = self.state + np.dot(K, Y)
        self.covariance = self.covariance - np.dot(K, np.dot(H, self.covariance))

def create_ekf():
    initial_state = [0, 0, 0, 0]  # Example: x, y, vx, vy
    initial_covariance = block_diag(100, 100, 10, 10)
    process_noise = block_diag(1, 1, 0.1, 0.1)
    measurement_noise = block_diag(5, 5)
    ekf = ExtendedKalmanFilter(initial_state, initial_covariance, process_noise, measurement_noise)
    return ekf
