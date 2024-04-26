## takes in accelerometer IMU and combines to output state to ROS code

from math import sin, cos, tan, pi
from enum import IntEnum
import numpy as np
from rclpy.node import Node  # Import Node from rclpy instead of using rospy
from geometry_msgs.msg import Vector3  # Messages are similar to ROS1

# State indices enum remains unchanged
class StateIndex(IntEnum):
    X = 0
    Y = 1
    Z = 2
    VX = 3
    VY = 4
    VZ = 5
    AX = 6
    AY = 7
    AZ = 8
    YAW = 9
    PITCH = 10
    ROLL = 11
    VYAW = 12
    VPITCH = 13
    VROLL = 14

# EKF class now inherits from Node
import numpy as np

class EKF(Node):
    NUM_FIELDS = 15

    def __init__(self, dvl_offset: np.array, process_covariance: np.array):
        super().__init__('ekf_node')
        self._dvl_offset = dvl_offset
        self._process_covariance = process_covariance
        self._state = np.zeros(EKF.NUM_FIELDS, float)
        self._covariance = 1e-9 * np.identity(EKF.NUM_FIELDS, float)
        self._time = None

    def _predict(self, timestamp):
        if self._time is None:
            self._time = timestamp
        else:
            delta_t = (timestamp.nanoseconds() - self._time.nanoseconds()) * 1e-9
            self._state = self._extrapolate_state(delta_t)
            self._covariance = self._extrapolate_covariance(delta_t)
            self._time = timestamp

    def _update(self, y, covariance, fields):
        H = self._get_H(fields)
        R = np.diag(covariance)
        S = H @ self._covariance @ H.T + R
        K = self._covariance @ H.T @ np.linalg.inv(S)
        self._state += K @ y
        I = np.identity(self.NUM_FIELDS)
        self._covariance = (I - K @ H) @ self._covariance

    def _extrapolate_state(self, delta_t):
        # Simplistic linear motion model for example purposes
        F = np.identity(self.NUM_FIELDS)
        return F @ self._state

    def _extrapolate_covariance(self, delta_t):
        F = np.identity(self.NUM_FIELDS)
        Q = delta_t * self._process_covariance  # Process noise scaled by time
        return F @ self._covariance @ F.T + Q

    def _get_H(self, fields):
        H = np.zeros((len(fields), self.NUM_FIELDS))
        for i, field in enumerate(fields):
            H[i, field] = 1
        return H

    def _wrap_angles(self, y, angle_indices):
        for index in angle_indices:
            while y[index] > np.pi:
                y[index] -= 2 * np.pi
            while y[index] < -np.pi:
                y[index] += 2 * np.pi
        return y

    #maybe dont need
    def get_state(self, time):
        if self._time is None or time < self._time:
            raise ValueError('get_state for time earlier than last time')

    # Correcting the time calculation by removing the parentheses from nanoseconds
        delta_t = (time.nanoseconds - self._time.nanoseconds) * 1e-9
        state = self._extrapolate_state(delta_t)

        position = Vector3(x=state[StateIndex.X], y=state[StateIndex.Y], z=state[StateIndex.Z])
        velocity = Vector3(x=state[StateIndex.VX], y=state[StateIndex.VY], z=state[StateIndex.VZ])
        acceleration = Vector3(x=state[StateIndex.AX], y=state[StateIndex.AY], z=state[StateIndex.AZ])
        orientation = Vector3(x=state[StateIndex.ROLL], y=state[StateIndex.PITCH], z=state[StateIndex.YAW])
        angular_velocity = Vector3(x=state[StateIndex.VROLL], y=state[StateIndex.VPITCH], z=state[StateIndex.VYAW])
        return position, velocity, acceleration, orientation, angular_velocity


    def handle_imu_measurement(self, orientation, linear_acceleration, covariance, timestamp):
        self._predict(timestamp)

        fields = [StateIndex.YAW, StateIndex.PITCH, StateIndex.ROLL, StateIndex.AX, StateIndex.AY, StateIndex.AZ]
        H = self._get_H(fields)
        z = np.concatenate((np.flip(orientation), linear_acceleration))
        y = z - H @ self._state
        y = self._wrap_angles(y, [0, 1, 2])

        self._update(y, covariance, fields)

    def handle_dvl_measurement(self, linear_velocity, covariance, timestamp):
        self._predict(timestamp)

        fields = [StateIndex.VX, StateIndex.VY, StateIndex.VZ]
        H = self._get_H(fields)
        z = linear_velocity - self._get_dvl_tangential_velocity()
        y = z - H @ self._state

        self._update(y, covariance, fields)
    
    def _get_dvl_tangential_velocity(self):
    # Assuming self._dvl_offset is the position offset of the DVL sensor from the vehicle's center of mass
    # and self._state contains angular velocity components at indices for VYAW, VPITCH, VROLL
    # and the orientation in PITCH and ROLL

    # Placeholder calculation - adjust based on your dynamics
        cp = np.cos(self._state[StateIndex.PITCH])
        sp = np.sin(self._state[StateIndex.PITCH])
        cr = np.cos(self._state[StateIndex.ROLL])
        sr = np.sin(self._state[StateIndex.ROLL])
        omega = np.array([self._state[StateIndex.VROLL], self._state[StateIndex.VPITCH], self._state[StateIndex.VYAW]])

    # Cross product of angular velocity and DVL offset to find tangential velocity
        tangential_velocity = np.cross(omega, self._dvl_offset)
        return tangential_velocity

