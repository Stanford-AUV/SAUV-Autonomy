import numpy as np
from spatialmath.base import *
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


def state_to_np(o: State): 
    if isinstance(o, State):
        return np.array(
            [
                o.position.x,
                o.position.y,
                o.position.z,
                o.orientation.x,
                o.orientation.y,
                o.orientation.z,
                o.orientation.w,
                o.linear_velocity.x,
                o.linear_velocity.y,
                o.linear_velocity.z,
                o.angular_velocity.x, 
                o.angular_velocity.y,
                o.angular_veloicty.z,
            ]
        )


def same_sgn(a: np.ndarray | float | int, b: np.ndarray | float | int) -> bool:
    if isinstance(a, np.ndarray) and isinstance(b, np.ndarray):
        return np.all(np.sign(a) == np.sign(b))
    elif isinstance(a, (float, int)) and isinstance(b, (float, int)):
        return np.sign(a) == np.sign(b)
    else:
        raise TypeError("Both arguments must be either numbers or numpy arrays.")
    

def clamp(value: float | int, limits: float | int) -> float | int:
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value


def axis_angle_from_quat(q):
    """Convert a unit quaternion to axis-angle representation"""
    s = q[0]
    v = q[1:4]
    normv = np.linalg.norm(v)
    if normv < 1e-10:
        return np.zeros(3)
    else:
        r = v / normv
        theta = 2 * np.arctan2(normv, s)
        return r * theta
    

def q_shorter(q):
    """Returns the equivalent quaternion with rotation < 180 degrees"""
    s = q[0]
    v = q[1:4]
    if s < 0:
        return -q
    else:
        return q
    

def get_axis_angle_error(q_WB, q_W_des):
    # desired attitude_q_current attitude
    q_des_B = qqmul(qconj(q_W_des), q_WB)

    # make sure this quat describes the <180 degree rotation
    # because q and -q both describe the same attitude, but
    # one is > 180, one is < 180
    q_des_B = q_shorter(q_des_B)

    des_phi_b = axis_angle_from_quat(q_des_B)

    return des_phi_b