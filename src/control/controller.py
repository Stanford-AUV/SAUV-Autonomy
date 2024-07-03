import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Empty
import numpy as np
import threading
from msgs.msg import Pose, State, Wrench
from geometry_msgs.msg import Vector3
from simple_pid import PID
from scipy.spatial.transform import Rotation
from spatialmath.base import *

from control.utils import state_to_np, same_sgn, clamp, get_axis_angle_error


class Controller(Node):

    def __init__(self, 
                 kP,
                 kI,
                 kD,
                 ff,
                 alpha = [1, 1, 1, 1, 1, 1],
                 max = None,
                 start_i = None,
                 time_fn = None,
    ):
        """
        Initialize Controller object

        :param kP: 6-vector of P gains for the controller.
        :param kI: 6-vector of I gains for the controller.
        :param kD: 6-vector of D gains for the controller.
        :param ff: 6-vector of feed forward terms for the controller.
        :param alpha: 6-vector of weights for the derivative term in the integral calculation, or None by default.
        :param max: 6-vector of max force or torque allowed in each coordinate, or the 1 vector by default.
        :param start_i: 6-vector representing what error to begin accumulating the integral term.
        :param time_fn: Function to keep track of time between state updates, or None by default.
        """
        # Gains for translational PID
        self.kP_trans = np.diag(kP[:3])
        self.kI_trans = np.diag(kI[:3])
        self.kD_trans = np.diag(kD[:3])
        self.ff_trans = ff[:3]
        
        # Gains for rotational PID
        self.kP_rot = np.diag(kP[3:])
        self.kI_rot = np.diag(kI[3:])
        self.kD_rot = np.diag(kD[3:])
        self.ff_rot = ff[3:]

        self.start_i = start_i

        self.dim = len(self.kP)

        self.current = np.zeros(self.dim)
        self.desired = np.zeros(self.dim)
        self.integral = np.zeros(self.dim)
        self.max = max

        self.time_fn = time_fn
        self.last_time = self.get_clock().now()

        self.state_subscription_ = self.create_subscription(State, "state", self.state_callback, 10)
        
        self.desired_subscription_ = self.create_subscription(Pose, "desired_pose", self.desired_callback, 10)

        self.output_publisher_ = self.create_publisher(Wrench, "desired_wrench", 10)

        self.reset_service = self.create_service(Empty, "reset_controller", self.reset)

        self.lock = threading.Lock()

    def state_callback(self, msg: State):
        """
        Get our current state from a topic.
        """
        with self.lock:
            self.current = state_to_np(msg)
            timestamp = Time(
                seconds = msg.header.stamp.sec,
                nanoseconds = msg.header.stamp.nanosec,
                clock_type = self.get_clock().clock_type
            )
            self.dt = (timestamp - self.last_time).nanoseconds
            self.last_time = timestamp
            self.update()

    def desired_callback(self, msg: State):
        """
        Get the desired state from a topic.
        """
        with self.lock:
            self.desired = state_to_np(msg)

    def update(self):
        """
        Update the controller with the current state and publish to a topic.
        """
        # current state
        r_W = self.current[0:3]     # position, world-frame
        q_WB = self.current[3:7]    # orientation, world-frame
        q_BW = qconj(q_WB)   
        v_W = self.current[7:10]    # velocity, world-frame
        w_B = self.current[10:13]   # angular velocity, body-frame

        # desired state
        r_W_des = self.desired[0:3]     # position, world-frame
        q_WB_des = self.desired[3:7]    # orientation, world-frame
        q_BW_des = qconj(q_WB_des)
        v_W_des = self.desired[7:10]    # velocity, world-frame
        w_B_des = self.desired[10:13]   # angular velocity, body-frame

        # error terms
        pos_error = r_W_des - r_W                          # position
        des_phi_b = get_axis_angle_error(q_WB, q_WB_des)   # attitude

        # derivative terms 
        d_v_W = v_W_des - v_W   # velocity
        d_w_B = w_B_des - w_B   # angular velocity

        # integral terms
        for i in range(len(self.integral)):
            if (i < 3) and (pos_error[i] < self.start_i[i]):
                self.integral[i] += self.dt * (pos_error[i] + self.alpha[i] * d_v_W[i])
            else:
                self.integral[i] += self.dt * (des_phi_b[i-3] + self.alpha[i] * d_w_B[i-3])
            clamp(self.integral[i], [self.integral[i], self.max[i]])
        
        # force on the body (in the body frame)
        force_W = self.kP_trans @ pos_error + self.kD_trans @ d_v_W + self.kI_trans @ self.integral[:3] + self.ff_trans
        force_B = qvmul(q_BW, force_W)

        # torque on the body (in the body frame)
        tau_B = self.kP_rot @ des_phi_b + self.kD_rot @ d_w_B + self.kI_rot @ self.integral[3:] + self.ff_rot

        wrench = np.hstack((force_B, tau_B))

        # Publish a list of control outputs:
        # [force_x, force_y, force_z, torque_roll, torque_pitch, torque_yaw]
        msg = Wrench(
            force=Vector3(x=wrench[0], y=wrench[1], z=wrench[2]),
            torque=Vector3(x=wrench[3], y=wrench[4], z=wrench[5]),
        )

        self.output_publisher_.publish(msg)

    def reset(self):
        """
        Reset the controller settings.
        """
        self.get_logger().info("Resetting controler settings.")
        with self.lock:
            self.current = np.zeros(self.dim)
            self.desired = np.zeros(self.dim)
            self.integral = np.zeros(self.dim)


# 0.01 -> 20.02
# 0.05 -> 20.04
# 0.1 -> 20.08
# 1 -> 20.11

# 0.1, 0.1 -> 15.09
# 0.1, 1 -> 15.07
# 0.1, 1000 -> 15.05
# 0.1, 1000000 ->


def main(args=None):
    rclpy.init(args=args)

    # Initialize controller gains
    #                      x, y, z, r, p, y
    kP = np.array([0.1, 0.1, 0.1, 0.05, 0.05, 0.05])
    kD = np.array([0, 0, 0, 0, 0, 0])
    kI = np.array([1.2, 1.2, 1.2, 0.75, 0.75, 0.75])
    start_I = np.array([0, 0, 0, 0, 0, 0])

    controller_node = Controller(kP, kD, kI, start_I)

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    controller_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()