import numpy as np
from control.ekf import EKF
from scipy.spatial.transform import Rotation


class ObjectEKF(EKF):
    def __init__(
        self,
        initial_state_covariance,
        initial_process_covariance,
        initial_measurement_covariances,
    ):
        super().__init__(6, initial_state_covariance, initial_process_covariance)
        self.R_obj = np.diag(initial_measurement_covariances["obj"])

    def predict(self, dt):
        self.F[0:3, 3:6] = np.eye(3) * dt  # Object position depends on velocity

        super().predict(dt)

    def handle_object_detection(
        self, obj_position_robot_frame, robot_position_world, robot_orientation_world
    ):
        R = Rotation.from_euler("xyz", robot_orientation_world)
        obj_position_world = robot_position_world + R.apply(obj_position_robot_frame)

        H = np.zeros((3, 6))
        H[:, 0:3] = np.eye(3)  # Object position measurement matrix

        self.update(H, self.R_obj, obj_position_world)

    def get_state(self):
        position = self.state[0:3]
        velocity = self.state[3:6]

        return position, velocity, self.P
