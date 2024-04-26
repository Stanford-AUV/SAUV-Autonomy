import unittest
import numpy as np
import rclpy
from rclpy.time import Time
from ekf import EKF, StateIndex  # Assuming EKF and StateIndex are in ekf.py

class TestEKF(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize ROS 2 client library only once before all tests
        rclpy.init()

    def setUp(self):
        # Initialize the EKF object with zero DVL offset and identity matrix for process covariance
        self.ekf = EKF(dvl_offset=np.zeros(3), process_covariance=np.identity(15))

        # Mock a timestamp
        self.timestamp = Time(seconds=int(1e9))  # 1e9 nanoseconds converted to a Time object

    def test_handle_imu_measurement(self):
        # Provide some dummy orientation and linear acceleration data
        orientation = np.array([0.0, 0.0, 0.0])  # Assume no initial rotation
        linear_acceleration = np.array([0.0, 0.0, -9.81])  # Gravity
        covariance = np.ones(6)  # Simplified covariance

        # Call the method under test
        self.ekf.handle_imu_measurement(orientation, linear_acceleration, covariance, self.timestamp)

        # Retrieve the state and validate the expected values
        position, velocity, acceleration, orientation, angular_velocity = self.ekf.get_state(self.timestamp)
        np.testing.assert_array_almost_equal(acceleration.x, 0.0)
        np.testing.assert_array_almost_equal(acceleration.y, 0.0)
        np.testing.assert_array_almost_equal(acceleration.z, -9.81)

    def test_handle_dvl_measurement(self):
        # Provide some dummy DVL data indicating velocity along x-axis
        velocity = np.array([1.0, 0.0, 0.0])  # 1 m/s in the x direction
        covariance = np.ones(3)  # Simplified covariance

        # Call the method under test
        self.ekf.handle_dvl_measurement(velocity, covariance, self.timestamp)

        # Retrieve the state and validate the expected values
        position, velocity, acceleration, orientation, angular_velocity = self.ekf.get_state(self.timestamp)
        np.testing.assert_array_almost_equal(velocity.x, 1.0)
        np.testing.assert_array_almost_equal(velocity.y, 0.0)
        np.testing.assert_array_almost_equal(velocity.z, 0.0)

    @classmethod
    def tearDownClass(cls):
        # Shut down ROS 2 client library after all tests
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()

