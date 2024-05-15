import numpy as np
from geometry_msgs.msg import Vector3, Quaternion, Point


def toNp(o) -> np.ndarray:
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
