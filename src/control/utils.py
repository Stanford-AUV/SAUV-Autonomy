import numpy as np
from geometry_msgs.msg import Vector3, Quaternion, Point
from msgs.msg import Pose, State, Wrench


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
