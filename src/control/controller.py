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

from control.utils import pose_to_np, vel_to_np, same_sgn


class Controller(Node):

    def __init__(self, 
                 p_values: np.ndarray,
                 i_values: np.ndarray,
                 d_values: np.ndarray,
                 start_i_values: np.ndarray,
                 max_i_values: np.ndarray,
                 dterm_i_values: np.ndarray
                ):
        """
        Initialize Controller with PID parameters for position and orientation [x, y, z, roll, pitch, yaw].

        :param p_values: Diagonal matrix of P gains.
        :param i_values: Diagonal matrix of I gains.
        :param d_values: Diagonal matrix of D gains.
        :param start_i_values: Diagonal matrix of errors to start integral term.
        :param max_i_values: Diagonal matrix of maximum integral values.
        :param dterm_i_values: Diagonal matrix of weights for the derivative term in the integral calculation
        """

        super().__init__("controller")

        # Initialize controller parameters
        self._kP = p_values
        self._kI = i_values
        self._kD = d_values
        self._start_i = start_i_values
        self._max_i = max_i_values
        self._alpha = dterm_i_values

        # Initialize time keeping variables
        self.timer = lambda: self.get_clock().now().nanoseconds / 1e9 
        self.last_time = self.get_clock().now()

        self._dim = 6   # 6 DOF     

        # Initialize controller mutables; everything is 0 to start
        self.current_pose = np.zeros(self._dim)
        self.desired_pose = np.zeros(self._dim)
        self.previous_pose = np.zeros(self._dim)
        self.current_vel = np.zeros(self._dim)
        self.desired_vel = np.zeros(self._dim)
        self.integral = np.zeros(self._dim)

        # Initialize current state subscriber
        self._state_subscription = self.create_subscription(State, "state", self.state_callback, 10)

        # Initialize desired state subscriber
        self._desired_subscription = self.create_subscription(Pose, "desired_pose", self.desired_callback, 10)

        # Initialize force/torque output publisher
        self.output_publisher_ = self.create_publisher(Wrench, "desired_wrench", 10)

        # Services
        self.reset_service = self.create_service(Empty, "reset_controller", self.reset)

        # Create mutex to prevent race conditions
        self.lock = threading.Lock()

    def state_callback(self, msg: State):
        """Get our current state from a topic and extract relevant information."""

        with self.lock: 
            # Get current pose and velocities
            self.pose = np.array(pose_to_np(msg))
            self.vel = np.array(vel_to_np(msg))
            # self.get_logger().info("Received pose: %s" % self.pose)
            # self.get_logger().info("Received vel: %s" % self.vel)

            # Get time interval from last reading
            timestamp = Time(
                seconds = msg.header.stamp.sec, 
                nanoseconds = msg.header.stamp.nanosec,
                clock_type = self.get_clock().clock_type
            )
            self.dt = (timestamp - self.last_time).nanoseconds   # time difference in nanoseconds
            self.last_time = timestamp

            self.update()

    def desired_callback(self, msg: Pose):
        """Get the desired pose and velocities from a topic."""

        with self.lock:
            # Get current pose and velocities
            self.desired_pose = np.array(pose_to_np(msg))
            self.desired_vel = np.array(vel_to_np(msg))
            # self.get_logger().info("Received desired pose: %s" % self.desired)
            # self.get_logger().info("Received desired pose: %s" % self.desired)

    def update(self):
        """Update the controller with the current state and publish to a topic"""

        # Calculate wrench in global frame
        pose_error = self.desired_pose - self.current_pose
        vel_error = self.desired_vel - self.current_vel
        overshoot = same_sgn(pose_error, self.previous_error)
        for i in range(len(self.current_pose)) :
            if pose_error[i] < self._start_i[i] and overshoot :
                self.integral[i] += (pose_error + self.alpha * vel_error) * (self.dt / 1e9)
            self.integral[i] = min(self.integral[i], self._max_i[i])
        global_wrench = (self._kP @ )

        # Convert wrench to local frame
        current_orientation = self.pose[3:]
        rotation_matrix = Rotation.from_euler("xyz", -current_orientation).as_matrix()
        local_wrench = np.concatenate(
            [rotation_matrix @ global_wrench[:3], rotation_matrix @ -global_wrench[3:]]
        )   

        np.set_printoptions(precision=2, suppress=True)
        self.get_logger().info(f"Local wrench: {local_wrench}")

        # Cap the wrench to prevent thrust from exceeding limits
        wrench = np.clip(local_wrench, -1, 1)
        for i in range(3, 6):
            if abs(wrench[i]) < 0.001:
                wrench[i] = 0

        # Publish a list of control outputs:
        # [force_x, force_y, force_z, torque_roll, torque_pitch, torque_yaw]
        msg = Wrench(
            force=Vector3(x=wrench[0], y=wrench[1], z=wrench[2]),
            torque=Vector3(x=wrench[3], y=wrench[4], z=wrench[5]),
        )

        self.output_publisher_.publish(msg)
        # self.get_logger().info(f"Publishing wrench: {wrench}")

    def reset(self): 
        """Reset the controller settings."""

        self.get_logger().info("Resetting controler settings.")
        with self.lock:
            self.current_pose = np.zeros(self._dim)
            self.desired_pose = np.zeros(self._dim)
            self.previous_pose = np.zeros(self._dim)
            self.current_vel = np.zeros(self._dim)
            self.desired_vel = np.zeros(self._dim)
            self.integral = np.zeros(self._dim)


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

    # Initialize controller gains [x, y, z, r, p, y]
    kP = np.array([0.1, 0.1, 0.1, 0.05, 0.05, 0.05])
    kD = np.array([0, 0, 0, 0, 0, 0])
    kI = np.array([1.5, 1.5, 1.5, 0.75, 0.75, 0.75])
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