import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Int16
from control.force_to_pwm import total_force_to_individual_pwm


class ThrusterManager(Node):

    def __init__(self):
        super().__init__("thruster_manager")

        self._thruster_ids = [f"thruster{i}" for i in range(1, 9)]

        self._pwm_pubs = []
        for thruster in self._thruster_ids:
            pwm_pub = self.create_publisher(Int16, f"thrusters/{thruster}/pwm", 10)
            self._pwm_pubs.append(pwm_pub)

        timer_period = 0.5  # TODO: Don't hardcode this
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        testing_wrench = np.array(
            [1, 0, 0, 0, 0, 0]
        )  # TODO: Actually get this based on where the robot wants to go, this only goes forward
        thrust_pwms = total_force_to_individual_pwm(testing_wrench)
        for i, pwm in enumerate(thrust_pwms.tolist()):
            msg = Int16()
            msg.data = pwm
            self._pwm_pubs[i].publish(msg)
            self.get_logger().info(f"Publishing thruster {i} PWM = {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ThrusterManager()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
