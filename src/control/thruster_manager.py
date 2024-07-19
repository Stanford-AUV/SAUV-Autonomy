import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Int16, Float64
from msgs.msg import Wrench
from control.force_to_pwm import total_force_to_individual_thrusts, all_thrusts_to_pwm
from control.utils import wrench_to_np


class ThrusterManager(Node):

    def __init__(self):
        super().__init__("thruster_manager")

        self._thruster_ids = [f"thruster_{i}" for i in range(1, 9)]

        self.wrench = np.zeros(6)

        self._pwm_pubs = []
        self._thrust_pubs = []
        for thruster in self._thruster_ids:
            pwm_pub = self.create_publisher(Int16, f"thrusters/{thruster}/pwm", 10)
            thrust_pub = self.create_publisher(
                Float64, f"thrusters/{thruster}/magnitude", 10
            )
            self._pwm_pubs.append(pwm_pub)
            self._thrust_pubs.append(thrust_pub)
        self._wrench_sub = self.create_subscription(
            Wrench, "desired_wrench", self.wrench_callback, 10
        )

        timer_period = 0.01  # TODO: Don't hardcode this
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        thrusts = total_force_to_individual_thrusts(self.wrench)
        pwms = all_thrusts_to_pwm(thrusts)
        for i, thrust in enumerate(thrusts.tolist()):
            msg = Float64()
            msg.data = thrust
            self._thrust_pubs[i].publish(msg)
        for i, pwm in enumerate(pwms.tolist()):
            msg = Int16()
            msg.data = pwm
            self._pwm_pubs[i].publish(msg)

    def wrench_callback(self, msg: Wrench):
        self.wrench = wrench_to_np(msg)
        self.get_logger().info(f"Received wrench {self.wrench}")


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
