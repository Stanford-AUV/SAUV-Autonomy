import numpy as np
import math
from geometry_msgs.msg import Vector3, Quaternion, Point
from nav_msgs.msg import Odometry
from msgs.msg import Pose, State, Wrench
from scipy.spatial.transform import Rotation

def to_np(o) -> np.ndarray:
    """Converts a ROS message to a numpy array."""
    if isinstance(o, Vector3):
        return np.array([o.x, o.y, o.z])
    if isinstance(o, Point):
        return np.array([o.x, o.y, o.z])
    if isinstance(o, Quaternion):
        return np.array([o.x, o.y, o.z, o.w])
    if isinstance(o, list):
        return np.array(o)
    raise ValueError("Unsupported type for toNp")


def pose_to_np(o: State | Pose):
    if isinstance(o, State) or isinstance(o, Pose) or isinstance(o, Wrench):
        return np.array(
            [
                o.position.x,
                o.position.y,
                o.position.z,
                o.orientation.x,
                o.orientation.y,
                o.orientation.z,
            ]
        )

def quaternion_to_euler(quaternion):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw) in radians
    """
    rot = Rotation.from_quat(quaternion)
    return rot.as_euler('xyz')
    

def euler_to_quaternion(euler_angle):
    """
    Convert Euler angles (roll, pitch, yaw) in radians to quaternions
    """
    rot = Rotation.from_euler(euler_angle)
    return rot.as_quat()


def state_to_np(o: State | Pose):
    if isinstance(o, State) or isinstance(o, Pose) or isinstance(o, Wrench):
        return np.array(
            [
                o.position.x,
                o.position.y,
                o.position.z,
                o.orientation_euler.x,
                o.orientation_euler.y,
                o.orientation_euler.z,
            ]
        )
    
def odometry_to_np(o: Odometry):
    if isinstance(o, Odometry):
        position = [
            o.pose.pose.position.x,
            o.pose.pose.position.y,
            o.pose.pose.position.z
        ]
        orientation_quat = [
            o.pose.pose.orientation.x,
            o.pose.pose.orientation.y,
            o.pose.pose.orientation.z,
            o.pose.pose.orientation.w
        ]
        orientation_euler = quaternion_to_euler(orientation_quat)
        position.extend(orientation_euler)
        return position
        


def wrench_to_np(o: Wrench):
    if isinstance(o, Wrench):
        return np.array(
            [
                o.force.x,
                o.force.y,
                o.force.z,
                o.torque.x,
                o.torque.y,
                o.torque.z,
            ]
        )
