import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Imu
from ros_gz_interfaces.msg import Altimeter
from geometry_msgs.msg import PoseStamped, Vector3
from msgs.msg import State

from control.utils import to_np


class GazeboManager(Node):

    def __init__(self):
        super().__init__("gazebo_manager")

        self._thruster_ids = [f"thruster_{i}" for i in range(1, 9)]

        self._gazebo_thrust_pubs = {}
        for thruster in self._thruster_ids:
            self.create_subscription(
                Float64,
                f"thrusters/{thruster}/magnitude",
                self.thruster_callback(thruster),
                10,
            )
            gazebo_thrust_pub = self.create_publisher(Float64, f"gz/{thruster}", 10)
            self._gazebo_thrust_pubs[thruster] = gazebo_thrust_pub

        self._imu_sub = self.create_subscription(Imu, "gz/imu", self.imu_callback, 10)
        self._imu_pub = self.create_publisher(Imu, "imu", 10)

        self._altimeter_sub = self.create_subscription(
            Altimeter, "gz/depth", self.altimeter_callback, 10
        )
        self._altimeter_pub = self.create_publisher(Altimeter, "depth", 10)

        self._pose_sub = self.create_subscription(
            PoseStamped, "gz/pose", self.pose_callback, 10
        )
        self._state_pub = self.create_publisher(State, "state", 10)

    def thruster_callback(self, thruster):
        def callback(msg):
            thrust = msg.data
            msg = Float64()
            msg.data = thrust * 100
            self._gazebo_thrust_pubs[thruster].publish(msg)
            self.get_logger().info(
                f"Publishing thruster {thruster} thrust = {msg.data}"
            )

        return callback

    def imu_callback(self, msg: Imu):
        self.get_logger().info("Received IMU message")
        # Since underwater, remove g from the acceleration
        msg.linear_acceleration.z -= 9.8
        self._imu_pub.publish(msg)

    def altimeter_callback(self, msg: Altimeter):
        self.get_logger().info("Received altimeter message")
        self._altimeter_pub.publish(msg)

    def pose_callback(self, msg: PoseStamped):
        self.get_logger().info("Received pose message")
        state = State()
        state.header = msg.header
        state.position = Vector3(
            x=msg.pose.position.x, y=msg.pose.position.y, z=msg.pose.position.z
        )
        state.orientation = msg.pose.orientation
        self.get_logger().info(f"Publishing state: {state}")
        self._state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
