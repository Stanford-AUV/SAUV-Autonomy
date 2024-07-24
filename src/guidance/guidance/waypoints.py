import rclpy
from rclpy.node import Node
import numpy as np
import math
import json
from msgs.msg import Pose, State, Wrench
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from control.utils import pose_to_np, state_to_np, wrench_to_np


class CheckpointManager(Node):

    def __init__(self):
        super().__init__("waypoints")

        with open('/home/selenas/SAUV/SAUV-Autonomy/src/guidance/data/waypoints.json', 'r') as f: # orin filepath
            waypoints = json.load(f)
        self._checkpoints = np.array(waypoints, dtype=np.float64)
        self._checkpoints_index = 0
        self._desired_pose = self._checkpoints[self._checkpoints_index]
        self._desired_pose_pub = self.create_publisher(Pose, "desired_pose", 10)
        self._desired_wrench_sub = self.create_subscription(Wrench, "desired_wrench", self.wrench_callback, 10)
        
        self.dim_ = 6
        self.wrench = np.zeros(self.dim_)
        self.pose = np.zeros(self.dim_)
        self._current_state_sub = self.create_subscription(
            State, "state", self.current_state_callback, 10
        )

        timer_period = 0.1  # TODO: Don't hardcode this
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def normalize_angle(self, angle): # TODO these are duplicate functions (originals in controls), fix later
        """Normalize the angle to be within the range of [-π, π]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def find_yaw_error(self, current_yaw, desired_yaw): # TODO duplicate function
        """Find closest angle, handling pi -> -pi wrapping"""
        cur_error = self.normalize_angle(self.normalize_angle(desired_yaw) - self.normalize_angle(current_yaw)) 
        sign = 1 if cur_error > 0 else -1
        wrapped_error = 2 * np.pi - np.abs(cur_error)

        if wrapped_error < cur_error:
            error = -1 * sign * wrapped_error
        else:
            error = cur_error

        return error

    def wrench_callback(self, msg: Wrench):
        self.wrench = wrench_to_np(msg)

    def timer_callback(self):
        msg = Pose(
            header=Header(stamp=self.get_clock().now().to_msg()),
            position=Vector3(
                x=self._desired_pose[0],
                y=self._desired_pose[1],
                z=self._desired_pose[2],
            ),
            orientation=Vector3(
                x=self._desired_pose[3],
                y=self._desired_pose[4],
                z=self._desired_pose[5],
            ),
        )
        # msg.data = pwm
        self._desired_pose_pub.publish(msg)

    def current_state_callback(self, msg: State):
        self.pose = np.array(state_to_np(msg))
        eps_position = 0.5 # TODO tune
        eps_angle = 0.1 # TODO tune

        position_error = np.linalg.norm(self.pose[:3] - self._desired_pose[:3])
        yaw_error = np.abs(self.find_yaw_error(self.pose[5], self._desired_pose[5]))

        pose_rounded = [round(x, 2) for x in self.pose]
        wrench_rounded = [round(x, 2) for x in self.wrench]
        self.get_logger().info(f"\n\nCurrent Pose: {pose_rounded}\nDesired Pose: {self._desired_pose}\nPosition Error: {position_error}\nYaw Error: {yaw_error}\nWrench: {wrench_rounded}\n")
        # x_error = current_pose[0] - self._desired_pose[0] # for specific waypoints

        # if yaw_error < eps_angle:
        if (yaw_error < eps_angle) and (position_error < eps_position):
            if self._checkpoints_index < len(self._checkpoints) - 1:
                self._checkpoints_index += 1
                self._desired_pose = self._checkpoints[self._checkpoints_index]


def main(args=None):
    rclpy.init(args=args)
    node = CheckpointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
