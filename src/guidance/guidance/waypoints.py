import rclpy
from rclpy.node import Node
import numpy as np

import json

from msgs.msg import Pose, State
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header


class CheckpointManager(Node):

    def __init__(self):
        super().__init__("waypoints")

        with open('/home/selenas/SAUV/SAUV-Autonomy/src/guidance/data/waypoints.json', 'r') as f: # orin filepath
            waypoints = json.load(f)
        self._checkpoints = np.array(waypoints, dtype=np.float64)
        self._checkpoints_index = 0
        self._desired_pose = self._checkpoints[self._checkpoints_index]

        self._current_state_sub = self.create_subscription(
            State, "state", self.current_state_callback, 10
        )
        self._desired_pose_pub = self.create_publisher(Pose, "desired_pose", 10)

        timer_period = 0.1  # TODO: Don't hardcode this
        self.timer = self.create_timer(timer_period, self.timer_callback)

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
        self.get_logger().info(f"Published desired_pose:\n[{msg.position.x}, {msg.position.y}, {msg.position.z}, {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}]")

    def current_state_callback(self, msg: State):
        current_pose = np.array(
            [
                msg.position.x,
                msg.position.y,
                msg.position.z,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
            ]
        )
        eps_position = 0.1 # TODO tune
        eps_angle = 2.25 # TODO tune

        position_error = np.linalg.norm(current_pose[:3] - self._desired_pose[:3])
        yaw_error = np.abs(current_pose[5] - self._desired_pose[5])
        # angle_error = np.dot(current_pose[3:], self._desired_pose[3:]) / (np.linalg.norm(current_pose[3:]) * np.linalg.norm(self._desired_pose[3:])) # cosine similarity
        self.get_logger().info(f"Position Error: {position_error}\nAngle Error: {yaw_error}")
        # x_error = current_pose[0] - self._desired_pose[0] # for specific waypoints

        # if x_error < eps_position:
        # if position_error < eps_position and angle_error < eps_angle:
        if (yaw_error < eps_angle) and (position_error < eps_position) :
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
