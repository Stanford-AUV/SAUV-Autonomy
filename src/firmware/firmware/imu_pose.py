import rclpy
import ms5837
import math

from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped, QuaternionStamped
import tf_transformations

class ImuPose(Node):

    def __init__(self):
        super().__init__("imu_pose")
        self.quat = self.create_subscription(QuaternionStamped, "/filter/quaternion",self.callback,10)
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, "/imu/pose", 10)
        self.get_logger().info(f"Running IMU pose publisher")

    def callback(self, msg):
        # Bundle the messages into a single synchronized message
        imu_pose = PoseWithCovarianceStamped()
        imu_pose.header.stamp = msg.header.stamp
        imu_pose.header.frame_id = "odom"

        imu_pose.pose.pose.orientation.x = msg.quaternion.x
        imu_pose.pose.pose.orientation.y = msg.quaternion.y
        imu_pose.pose.pose.orientation.z = msg.quaternion.z
        imu_pose.pose.pose.orientation.w = msg.quaternion.w
       
        # Publish the synchronized message
        self.publisher_.publish(imu_pose)


def main(args=None):
    rclpy.init(args=args)

    imu_pose = ImuPose()

    rclpy.spin(imu_pose)

    imu_pose.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
