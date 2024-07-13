import numpy as np
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation
from rclpy.time import Time

class EKF:
    def __init__(self, dvl_offset, initial_process_covariance, initial_measurement_covariances):
        self.dvl_offset = dvl_offset
        self.process_covariance = np.diag(initial_process_covariance)

        self.state = np.zeros(15)  # State vector
        self.P = np.eye(15)  # State covariance matrix

        self.F = np.eye(15)  # State transition matrix
        self.Q = np.diag(initial_process_covariance)  # Process noise covariance matrix

        # Measurement noise covariances
        self.R_imu = np.diag(initial_measurement_covariances['imu'])
        self.R_dvl = np.diag(initial_measurement_covariances['dvl'])
        self.R_depth = np.array([[initial_measurement_covariances['depth']]])

        # Buffers for dynamic covariance updates
        self.imu_buffer = []
        self.dvl_buffer = []
        self.depth_buffer = []
        self.max_buffer_size = 100  # Buffer size for measurement covariance updates

    def predict(self, dt):
        # Update state transition matrix F with dt
        self.F[0:3, 3:6] = np.eye(3) * dt  # Position depends on velocity
        self.F[3:6, 6:9] = np.eye(3) * dt  # Velocity depends on acceleration

        # Predict state
        self.state = self.F @ self.state

        # Predict state covariance
        self.P = self.F @ self.P @ self.F.T + self.Q

        # print(f"Predicted state: {self.state}")

    def update_measurement_covariance(self, buffer, R, new_measurement):
        # Convert new_measurement to a 2D array if it is a single value
        new_measurement = np.atleast_2d(new_measurement).T if np.isscalar(new_measurement) else np.atleast_2d(new_measurement)
        buffer.append(new_measurement)
        if len(buffer) > self.max_buffer_size:
            buffer.pop(0)
        if len(buffer) > 1:
            measurements = np.vstack(buffer)
            updated_R = np.cov(measurements, rowvar=False)
            if R.shape == (1, 1):  # Handle scalar case for depth
                R[0, 0] = updated_R.item()
            else:
                R[:updated_R.shape[0], :updated_R.shape[1]] = updated_R

    def handle_imu_measurement(self, orientation, linear_acceleration, covariance, timestamp: Time):
        # Update orientation state
        self.state[9:12] = orientation

        # Convert linear acceleration to global frame
        R = Rotation.from_euler('xyz', orientation)
        global_accel = R.apply(linear_acceleration)
        self.state[6:9] = global_accel

        # Update the IMU measurement covariance dynamically
        imu_measurement = np.hstack([orientation, linear_acceleration])
        self.update_measurement_covariance(self.imu_buffer, self.R_imu, imu_measurement)

        # Measurement model
        R = self.R_imu
        H = np.zeros((6, 15))
        H[0:3, 9:12] = np.eye(3)  # Orientation measurement matrix
        H[3:6, 6:9] = np.eye(3)  # Acceleration measurement matrix

        # Innovation covariance
        S = H @ self.P @ H.T + R
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Actual measurement vector
        y = imu_measurement
        # Predicted measurement
        y_hat = H @ self.state

        # Innovation (measurement residual)
        innovation = y - y_hat

        # Update state estimate with the innovation
        self.state = self.state + K @ innovation
        # Update state covariance
        self.P = (np.eye(15) - K @ H) @ self.P

        # print(f"Updated state with IMU: {self.state}")

    def handle_dvl_measurement(self, velocity, covariance, timestamp: Time):
        # print(f"Handling DVL measurement at {timestamp}: velocity={velocity}")

        # Update the DVL measurement covariance dynamically
        self.update_measurement_covariance(self.dvl_buffer, self.R_dvl, velocity)

         # Convert DVL velocity to global frame
        orientation = self.state[9:12]
        R = Rotation.from_euler('xyz', orientation)
        global_velocity = R.apply(velocity)

        H = np.zeros((3, 15))
        H[:, 3:6] = np.eye(3)  # Linear velocity measurement matrix

        R = self.R_dvl

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        y_hat = H @ self.state

        # Innovation (measurement residual)
        innovation = velocity - y_hat

        self.state = self.state + K @ innovation
        self.P = (np.eye(15) - K @ H) @ self.P

        # print(f"Updated state with DVL: {self.state}")

    def handle_depth_measurement(self, depth, covariance, timestamp: Time):
        # print(f"Handling Depth measurement at {timestamp}: depth={depth}")

        # Update the Depth measurement covariance dynamically
        self.update_measurement_covariance(self.depth_buffer, self.R_depth, depth)

        H = np.zeros((1, 15))
        H[0, 2] = 1  # Depth measurement matrix

        R = self.R_depth

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        y_hat = H @ self.state

        # Innovation (measurement residual)
        innovation = depth - y_hat

        self.state = self.state + K @ innovation
        self.P = (np.eye(15) - K @ H) @ self.P

        # print(f"Updated state with Depth: {self.state}")

    def get_state(self, time):
        position = self.state[0:3]
        velocity = self.state[3:6]
        acceleration = self.state[6:9]
        orientation = self.state[9:12]
        angular_velocity = self.state[12:15]

        return position, velocity, acceleration, orientation, angular_velocity
