import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Empty
import numpy as np
import threading
from msgs.msg import Pose, State, Wrench
from geometry_msgs.msg import Vector3
from simple_pid import PID
from scipy.spatial.transform import Rotation as R
from guidance.trapezoidal_motion_profile import TrapezoidalMotionProfile
import json
import json

from control.utils import pose_to_np, state_to_np


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

        self.start_I_ = start_i_values

        self.pids = [
            PID(
                p,
                i,
                d,
                setpoint=0,
                sample_time=None,
                time_fn=lambda: self.get_clock().now().nanoseconds / 1e9,
            )
            for p, i, d in zip(p_values, i_values, d_values)
        ]

        self.dim_ = 6  # 6 DOF

        # Initialize mutables; everything is 0 at t_0
        self.desired = np.zeros(self.dim_)
        self.pose = np.zeros(self.dim_)

        self.last_time = self.get_clock().now()

        # Initialize current state subscriber
        self.state_subscription_ = self.create_subscription(
            State, "state", self.state_callback, 10
        )

        # Initialize desired state subscriber
        self.desired_subscription_ = self.create_subscription(
            Pose, "desired_pose", self.desired_pose_callback, 10
        )

        # Initialize force/torque output publisher
        self.output_publisher_ = self.create_publisher(Wrench, "desired_wrench", 10)

        # Services
        self.reset_service = self.create_service(Empty, "reset_controller", self.reset)

        # Create mutex to prevent race conditions
        self.lock = threading.Lock()

        # Motion profiles
        self.motion_profiles = [None] * self.dim_
        self.profile_start_time = None

    def state_callback(self, msg: State):
        """Get our current pose from a topic."""
        with self.lock:
            self.pose = np.array(state_to_np(msg))
            self.get_logger().info("Current state: %s" % self.pose)
            timestamp = Time(
                seconds=msg.header.stamp.sec,
                nanoseconds=msg.header.stamp.nanosec,
                clock_type=self.get_clock().clock_type,
            )
            self.dt = (timestamp - self.last_time).nanoseconds
            self.last_time = timestamp
            self.update()

    def desired_pose_callback(self, msg: Pose):
        """Get the desired pose from a topic."""
        with self.lock:
            self.desired = np.array(pose_to_np(msg))

            self.profile_start_time = self.get_clock().now().nanoseconds / 1e9

            max_vel = 1.0
            max_acc = 0.5
            dt = 0.1
            
            # self.get_logger().info("Desired pose: %s" % self.desired)

            """
            self.motion_profiles = [
                TrapezoidalMotionProfile(self.pose[i], self.desired[i], max_vel, max_acc, dt)
                for i in range(self.dim_)
            ]
            """

    def update(self):
        """Update the controller with the current state and publish to a topic"""

        current_time = self.get_clock().now().nanoseconds / 1e9

        """
        if self.profile_start_time is not None:

            elapsed_time = current_time - self.profile_start_time

            for i in range(self.dim_):

                if self.motion_profiles[i] is not None:

                    self.pids[i].setpoint = self.motion_profiles[i].get_desired_position(elapsed_time)
        """
        for i in range(self.dim_):
            self.pids[i].setpoint = self.desired[i]

        # Wrench in global frame
        global_wrench = np.array([pid(self.pose[i]) for i, pid in enumerate(self.pids)])

        # Convert wrench to local frame
        current_orientation = self.pose[3:]
        rotation_matrix = R.from_euler("xyz", -current_orientation).as_matrix()
        local_wrench = np.concatenate(
            [rotation_matrix @ global_wrench[:3], rotation_matrix @ -global_wrench[3:]]
        )

        np.set_printoptions(precision=2, suppress=True)
        # self.get_logger().info(f"Local wrench: {local_wrench}")

        # Cap the wrench to prevent thrust from exceeding limits
        wrench = np.clip(local_wrench, -1, 1)
        for i in range(3, 6):
            if abs(wrench[i]) < 0.001:
                wrench[i] = 0

        # Publish a list of control outputs:
        # [force_x, force_y, force_z, torque_roll, torque_pitch, torque_yaw]
        wrench *= -1
        msg = Wrench(
            force=Vector3(x=wrench[0], y=wrench[1], z=wrench[2]),
            torque=Vector3(x=wrench[3], y=wrench[4], z=wrench[5]),
        )

        self.output_publisher_.publish(msg)
        # self.get_logger().info(f"Publishing wrench: {wrench}")

    def reset(self):
        """
        Reset the controller settings.
        """
        self.get_logger().info("Resetting controller settings.")
        with self.lock:
            for pid, start_i in zip(self.pids, self.start_I_):
                pid.reset()
                pid.set_auto_mode(True, last_output=start_i)

def main(args=None):
    rclpy.init(args=args)

    # Initialize controller gains
    #                      x, y, z, r, p, y

    with open('/home/selenas/SAUV/SAUV-Autonomy/src/control/data/pid_gains.json', 'r') as f:
        pid_gains = json.load(f)

    kP = np.array(pid_gains['kP'])
    kD = np.array(pid_gains['kD'])
    kI = np.array(pid_gains['kI'])
    start_I = np.array(pid_gains['start_I'])

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
