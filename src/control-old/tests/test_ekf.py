import numpy as np
import unittest
from rclpy.time import Time

from control.ekf import EKF, StateIndex


# TODO: Add more tests for EKF since the current ones are not very comprehensive
class TestEKF(unittest.TestCase):
    def setUp(self):
        dvl_offset = np.array([0.1, 0.0, 0.1])
        process_covariance = np.diag([0.01] * EKF.NUM_FIELDS)
        self.ekf = EKF(dvl_offset, process_covariance)
        self.initial_time = Time(seconds=0)

    def test_initial_state(self):
        self.assertTrue(np.array_equal(self.ekf._state, np.zeros(EKF.NUM_FIELDS)))
        self.assertTrue(
            np.allclose(self.ekf._covariance, 1e-9 * np.identity(EKF.NUM_FIELDS))
        )

    def test_state_extrapolation(self):
        self.ekf._state[StateIndex.X : StateIndex.AZ + 1] = np.random.random(9)
        self.ekf._state[StateIndex.YAW : StateIndex.VROLL + 1] = (
            np.pi * np.random.random(6) - np.pi / 2
        )
        dt = 0.1
        new_time = Time(seconds=dt)
        self.ekf._time = self.initial_time

        old_state = self.ekf._state.copy()
        self.ekf._predict(new_time)
        expected_state = self.ekf._extrapolate_state(dt)

        np.testing.assert_allclose(self.ekf._state, expected_state, atol=1)
        self.assertNotEqual(old_state.tolist(), self.ekf._state.tolist())

    def test_imu_handling(self):
        orientation = np.array([0.1, 0.2, 0.3])
        linear_acceleration = np.array([0.01, -0.02, 9.81])
        covariance = np.ones(6) * 0.05
        timestamp = Time(seconds=1)

        self.ekf.handle_imu_measurement(
            orientation, linear_acceleration, covariance, timestamp
        )
        # Assert that state is updated, test details depend on _predict and _update implementation
        self.assertNotEqual(np.sum(self.ekf._state), 0)

    def test_dvl_handling(self):
        linear_velocity = np.array([1.0, -1.0, 0.5])
        covariance = np.ones(3) * 0.05
        timestamp = Time(seconds=2)

        self.ekf.handle_dvl_measurement(linear_velocity, covariance, timestamp)
        # Similar assertion about state change
        self.assertNotEqual(np.sum(self.ekf._state), 0)

    def test_depth_handling(self):
        depth = np.array([10.0])
        covariance = np.array([1e-10])
        timestamp = Time(seconds=3)

        self.ekf.handle_depth_measurement(depth, covariance, timestamp)
        # Assert changes specific to depth handling
        self.assertAlmostEqual(self.ekf._state[StateIndex.Z], 10.0, delta=1)

    def test_angle_wrapping(self):
        angles = np.array([3 * np.pi, -3 * np.pi, np.pi / 2])
        wrapped = self.ekf._wrap_angles(angles, [0, 1, 2])
        np.testing.assert_allclose(wrapped, [-np.pi, -np.pi, np.pi / 2])


if __name__ == "__main__":
    unittest.main()
