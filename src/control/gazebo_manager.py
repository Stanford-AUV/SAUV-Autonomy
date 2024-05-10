import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class GazeboManager(Node):

    def __init__(self):
        super().__init__("gazebo_manager")

        self._thruster_ids = [f"thruster_{i}" for i in range(1, 9)]

        self._gazebo_thrust_pubs = {}
        for thruster in self._thruster_ids:
            self.create_subscription(
                Float64,
                f"thrusters/{thruster}/magnitude",
                self.listener_callback(thruster),
                10,
            )
            gazebo_thrust_pub = self.create_publisher(Float64, f"gz/{thruster}", 10)
            self._gazebo_thrust_pubs[thruster] = gazebo_thrust_pub

    def listener_callback(self, thruster):
        def callback(msg):
            thrust = msg.data
            msg = Float64()
            msg.data = thrust * 100
            self._gazebo_thrust_pubs[thruster].publish(msg)
            self.get_logger().info(
                f"Publishing thruster {thruster} thrust = {msg.data}"
            )

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = GazeboManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
