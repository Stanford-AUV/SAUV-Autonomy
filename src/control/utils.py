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


def pose_to_np(o: Pose | State):
    if isinstance(o, Pose) or isinstance(o, State) or isinstance(o, Wrench):
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
    

def vel_to_np(o: State) :
    return np.array(
        [
            o.linear_velocity.x,
            o.linear_velocity.y,
            o.linear_velocity.z,
            o.euler_velocity.x,
            o.euler_velocity.y,
            o.euler_velocity.z,
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

def same_sgn(a: np.ndarray | float | int, b: np.ndarray | float | int) -> bool:
    if isinstance(a, np.ndarray) and isinstance(b, np.ndarray):
        return np.all(np.sign(a) == np.sign(b))
    elif isinstance(a, (float, int)) and isinstance(b, (float, int)):
        return np.sign(a) == np.sign(b)
    else:
        raise TypeError("Both arguments must be either numbers or numpy arrays.")