import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Empty
import numpy as np
import threading
from msgs.msg import Pose, State, Wrench
from geometry_msgs.msg import Vector3

from control.utils import pose_to_np


class Controller(Node):
    def __init__(self, p_values, i_values, d_values, start_i_values):
        """
        Initialize Controller object with PID parameters for position and
        orientation.

        :param p_values: Diagonal matrix of P gains for the controller
            [pX, pY, pZ, pRoll, pPitch, pYaw].
        :param i_values: Diagonal matrix of I gains for the controller
            [iX, iY, iZ, iRoll, iPitch, iYaw].
        :param d_values: Diagonal matrix of D gains for the controller
            [dX, dY, dZ, dRoll, dPitch, dYaw].
        :param start_i_values: Diagonal matrix of values to start integral for the
            controller [startX, startY, startZ, startRoll, startPitch, startYaw].
        """
        super().__init__("controller")

        self.kP_ = p_values
        self.kI_ = i_values
        self.kD_ = d_values
        self.start_I_ = start_i_values

        self.dim_ = 6  # 6 DOF

        # Initialize mutables; everything is 0 at t_0
        self.integral = np.zeros(self.dim_)
        self.derivative = np.zeros(self.dim_)
        self.desired = np.zeros(self.dim_)
        self.prev_error = np.zeros(self.dim_)
        self.pose = np.zeros(self.dim_)

        self.last_time = self.get_clock().now()

        # Initialize current state subscriber
        self.state_subscription_ = self.create_subscription(
            State, "state", self.state_callback, 10
        )

        # Initialize desired state subscriber
        self.desired_subscription_ = self.create_subscription(
            Pose, "desired_pose", self.desired_callback, 10
        )

        # Initialize force/torque output publisher
        self.output_publisher_ = self.create_publisher(Wrench, "desired_wrench", 10)

        # Services
        self.reset_service = self.create_service(Empty, "reset_controller", self.reset)

        # Create mutex to prevent race conditions
        self.running = False
        self.lock = threading.Lock()

    def state_callback(self, msg: State):
        """Get our current pose from a topic."""
        with self.lock:
            self.pose = np.array(pose_to_np(msg))
            # self.get_logger().info("Received pose: %s" % self.pose)
            timestamp = Time(
                seconds=msg.header.stamp.sec,
                nanoseconds=msg.header.stamp.nanosec,
                clock_type=self.get_clock().clock_type,
            )
            self.dt = (timestamp - self.last_time).nanoseconds
            self.last_time = timestamp
            self.update()

    def desired_callback(self, msg: Pose):
        """Get the desired pose from a topic."""
        with self.lock:
            self.desired = np.array(pose_to_np(msg))
            # self.get_logger().info("Received desired pose: %s" % self.desired)

    def update(self):
        """Update the controller with the current state and publish to a topic"""
        error = self.desired - self.pose
        self.get_logger().info("Received error: %s" % error)
        self.derivative = (error - self.prev_error) / self.dt

        self._logger.info(f"Received derivative: {self.derivative}")

        # Pose is represented as a vector, so we loop through all elements to
        # incorporate integral term
        for i in range(self.dim_):
            # Delay integral term to avoid integral wind-up
            if error[i] <= self.start_I_[i][i]:
                self.integral[i] += error[i] * self.dt

        # Wrench is a linear combination of error, derivative, and integral vectors
        uncapped_wrench = (
            self.kP_ @ error + self.kI_ @ self.integral + self.kD_ @ self.derivative
        )

        # Cap the wrench to prevent thrust from exceeding limits
        wrench = np.clip(uncapped_wrench, -1, 1)

        # Publish a list of control outputs:
        # [force_x, force_y, force_z, torque_roll, torque_pitch, torque_yaw]
        msg = Wrench(
            force=Vector3(x=wrench[0], y=wrench[1], z=wrench[2]),
            torque=Vector3(x=wrench[3], y=wrench[4], z=wrench[5]),
        )
        self.output_publisher_.publish(msg)
        np.set_printoptions(precision=2, suppress=True)
        self.get_logger().info(f"Publishing wrench: {wrench}")

        # Current error becomes previous error in the next time step
        self.prev_error = error

    def reset(self, request, response):
        """
        Reset the controller settings.

        :param request: The request message sent by the service (if any).
        :param response: The response message sent to the service (if any).
        :return response: The response, indicating that reset is successful.
        """
        self.get_logger().info("Resetting controler settings.")
        with self.lock:
            self.integral = 0
            self.derivative = 0
            self.prev_error = 0
            return response


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
    kP = np.diag(np.array([0.1, 0, 0, 0, 0, 0]))
    kD = np.diag(np.array([1000000, 0, 0, 0, 0, 0]))
    kI = np.diag(np.array([0, 0, 0, 0, 0, 0]))
    start_I = np.diag(np.array([0, 0, 0, 0, 0, 0]))

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
