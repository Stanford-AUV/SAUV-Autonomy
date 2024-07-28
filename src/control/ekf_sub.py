import numpy as np
from control.ekf import EKF


class SubEKF(EKF):
    def __init__(
        self,
        initial_state_covariance,
        initial_process_covariance,
        initial_measurement_covariances,
    ):
        super().__init__(12, initial_state_covariance, initial_process_covariance)

        self.R_imu = np.diag(initial_measurement_covariances["imu"])
        self.R_dvl = np.diag(initial_measurement_covariances["dvl"])

    def predict(self, dt):
        self.F[0:3, 6:9] = np.eye(3) * dt  # Position depends on velocity
        self.F[3:6, 9:12] = np.eye(3) * dt  # Orientation depends on angular velocity

        super().predict(dt)

    def handle_imu_measurement(self, orientation, linear_acceleration):
        H = np.zeros((6, 12))
        H[0:3, 3:6] = np.eye(3)  # Orientation measurement matrix
        H[3:6, 6:9] = np.eye(3)  # Acceleration measurement matrix

        imu_measurement = np.hstack([orientation, linear_acceleration])
        self.update(H, self.R_imu, imu_measurement)

    def handle_dvl_measurement(self, velocity):
        H = np.zeros((3, 12))
        H[:, 6:9] = np.eye(3)  # Linear velocity measurement matrix

        self.update(H, self.R_dvl, velocity)

    def get_state(self):
        position = self.state[0:3]
        orientation = self.state[3:6]
        velocity = self.state[6:9]
        angular_velocity = self.state[9:12]

        return position, orientation, velocity, angular_velocity
