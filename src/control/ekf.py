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
class EKF(Node):
    NUM_FIELDS = 15

    def __init__(self, dvl_offset: np.array, process_covariance: np.array):
        super().__init__('ekf_node')  # Initialize as a ROS2 node
        self._dvl_offset = dvl_offset
        self._process_covariance = process_covariance
        self._state = np.zeros(EKF.NUM_FIELDS, float)
        self._covariance = 1e-9 * np.identity(EKF.NUM_FIELDS, float)
        self._time = None  # Use rclpy.time.Time if needed

    def get_state(self, time):
        if self._time is None or time <= self._time:
            raise ValueError('get_state for time earlier than last time')

        delta_t = time.nanoseconds() * 1e-9 - self._time.nanoseconds() * 1e-9  # Convert nanoseconds to seconds

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
