import unittest
import rclpy
from rclpy.time import Time
import numpy as np
from math import radians
from geometry_msgs.msg import Vector3
from ekf_module import EKF, StateIndex  # Assuming your EKF class is in a file named ekf_module.py

def create_imu_data(orientation, linear_acceleration):
    """ Generate fake IMU data with given orientation (in degrees) and linear acceleration. """
    rad_orientation = np.radians(orientation)  # Convert degrees to radians
    return rad_orientation, linear_acceleration, np.ones(6)  # covariance matrix simplified as identity

def create_dvl_data(velocity):
    """ Generate fake DVL data with given velocity. """
    return velocity, np.ones(3)  # covariance matrix simplified as identity

class TestEKF(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=None)  # Initialize ROS 2
        self.ekf = EKF(dvl_offset=np.array([0.1, 0.0, 0.0]), process_covariance=np.identity(15))
        self.start_time = Time().now()  # Get current ROS 2 time

    def test_imu_integration(self):
        """ Test IMU data handling. """
        orientation = [0, 0, 0]  # yaw, pitch, roll in degrees
        linear_acceleration = [0, 0, -9.81]  # gravity vector in m/s^2
        imu_data = create_imu_data(orientation, linear_acceleration)
        
        # Simulate receiving an IMU measurement at current time
        self.ekf.handle_imu_measurement(*imu_data, self.start_time)

        # Check the state update
        position, velocity, acceleration, orientation, angular_velocity = self.ekf.get_state(self.start_time)
        self.assertAlmostEqual(orientation.z, 0)
        self.assertAlmostEqual(acceleration.z, -9.81)

    def test_dvl_integration(self):
        """ Test DVL data handling. """
        velocity = [1, 0, 0]  # Moving forward at 1 m/s
        dvl_data = create_dvl_data(velocity)
        
        # Simulate receiving a DVL measurement at current time
        self.ekf.handle_dvl_measurement(*dvl_data, self.start_time)

        # Check the state update
        position, velocity, acceleration, orientation, angular_velocity = self.ekf.get_state(self.start_time)
        self.assertAlmostEqual(velocity.x, 1)

    def tearDown(self):
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
