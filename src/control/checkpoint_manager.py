import rclpy
from rclpy.node import Node
import numpy as np

import json

from msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header


class CheckpointManager(Node):

    def __init__(self):
        super().__init__("checkpoint_manager")

        with open("data/checkpoints.json") as f:
            self._checkpoints = json.load(f)
        self._desired_pose = np.array(self._checkpoints[0])

        self._current_pose_sub = self.create_subscription(
            Pose, "pose", self.current_pose_callback, 10
        )
        self._desired_pose_pub = self.create_publisher(Pose, "desired_pose", 10)

        timer_period = 0.5  # TODO: Don't hardcode this
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
        self.get_logger().info(f"Published desired pose {msg}")

    def current_pose_callback(self, msg):
        current_pose = np.array([msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw])
        if np.linalg.norm(current_pose - self._desired_pose) < 0.1:
            self._checkpoints.pop(0)
            if len(self._checkpoints) == 0:
                self.destroy_node()
                rclpy.shutdown()
                return
            self._desired_pose = np.array(self._checkpoints[0])


def main(args=None):
    rclpy.init(args=args)
    node = CheckpointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
