import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer
from scipy.spatial.transform import Rotation as R

ACCEPTABLE_SLOP = (
    0.001  # message arrival tolerance (s) -- NOTE: the IMU is running at 100 Hz.
)


class SyncFilterExact(Node):

    def __init__(self):
        super().__init__("sync_filter_exact")

        self.sync_dvl_publisher_ = self.create_publisher(
            TwistWithCovarianceStamped, "/dvl/twist_sync", 10
        )
        self.sync_imu_publisher_ = self.create_publisher(
            Imu, "/imu/data_sync", 10
        )
        self.sync_imu_pose_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "/imu/pose_sync", 10
        )
        self.sync_depth_publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "/depth/pose_sync", 10
        )
        self.last_imu_sync_ts_sec = math.nan

        self.dvl_sub = Subscriber(self, TwistWithCovarianceStamped, "/dvl/twist")
        self.imu_sub = Subscriber(self, Imu, "/imu/data")
        self.depth_sub = Subscriber(self, PoseWithCovarianceStamped, "/depth/pose")
        self.imu_only_sub = self.create_subscription(
            Imu, "/imu/data", self.sync_callback_D, 10
        )

        # synchronize DVL, IMU, Depth data
        self.ts_caseA = ApproximateTimeSynchronizer(  # case A: all messages available
            [self.dvl_sub, self.imu_sub, self.depth_sub],
            queue_size=10,
            slop=ACCEPTABLE_SLOP,
        )
        self.ts_caseA.registerCallback(self.sync_callback_A)

        self.ts_caseB = ApproximateTimeSynchronizer(  # case B: IMU DVL available
            [self.dvl_sub, self.imu_sub], queue_size=10, slop=ACCEPTABLE_SLOP
        )
        self.ts_caseB.registerCallback(self.sync_callback_B)

        self.ts_caseC = ApproximateTimeSynchronizer(  # case C: IMU Depth available
            [self.imu_sub, self.depth_sub], queue_size=10, slop=ACCEPTABLE_SLOP
        )
        self.ts_caseC.registerCallback(self.sync_callback_C)

        self.get_logger().info(f"Running IMU/DVL/Depth data synchronization")

    def sync_callback_A(self, dvl_msg, imu_msg, depth_msg):
        # self.get_logger().info(f"A")
        self.last_imu_sync_ts_sec = (
            imu_msg.header.stamp
        )  # update most recent IMU time for valid full sync
        dvl_msg.header.stamp = self.last_imu_sync_ts_sec
        imu_msg.header.stamp = self.last_imu_sync_ts_sec

        imu_pose_msg = PoseWithCovarianceStamped()
        imu_pose_msg.header.stamp = self.last_imu_sync_ts_sec
        imu_pose_msg.header.frame_id = "odom"
        imu_pose_msg.pose.pose.orientation.x = imu_msg.orientation.x
        imu_pose_msg.pose.pose.orientation.y = imu_msg.orientation.y
        imu_pose_msg.pose.pose.orientation.z = imu_msg.orientation.z
        imu_pose_msg.pose.pose.orientation.w = imu_msg.orientation.w

        depth_msg.header.stamp = self.last_imu_sync_ts_sec

        self.sync_dvl_publisher_.publish(dvl_msg)
        self.sync_imu_publisher_.publish(imu_msg)
        self.sync_imu_pose_publisher_.publish(imu_pose_msg)
        self.sync_depth_publisher_.publish(depth_msg)

    def sync_callback_B(self, dvl_msg, imu_msg):
        # self.get_logger().info(f"B")
        CurrTS = imu_msg.header.stamp
        if (CurrTS != self.last_imu_sync_ts_sec):  # if time is the same as full sync, ignore since we've already pub'd
            dvl_msg.header.stamp = CurrTS
            imu_msg.header.stamp = CurrTS

            imu_pose_msg = PoseWithCovarianceStamped()
            imu_pose_msg.header.stamp = CurrTS
            imu_pose_msg.header.frame_id = "odom"
            imu_pose_msg.pose.pose.orientation.x = imu_msg.orientation.x
            imu_pose_msg.pose.pose.orientation.y = imu_msg.orientation.y
            imu_pose_msg.pose.pose.orientation.z = imu_msg.orientation.z
            imu_pose_msg.pose.pose.orientation.w = imu_msg.orientation.w

            self.sync_dvl_publisher_.publish(dvl_msg)
            self.sync_imu_publisher_.publish(imu_msg)
            self.sync_imu_pose_publisher_.publish(imu_pose_msg)

    def sync_callback_C(self, imu_msg, depth_msg):
        # self.get_logger().info(f"C")
        CurrTS = imu_msg.header.stamp
        if (CurrTS != self.last_imu_sync_ts_sec):  # if time is the same as full sync, ignore since we've already pub'd
            imu_msg.header.stamp = CurrTS

            imu_pose_msg = PoseWithCovarianceStamped()
            imu_pose_msg.header.stamp = CurrTS
            imu_pose_msg.header.frame_id = "odom"
            imu_pose_msg.pose.pose.orientation.x = imu_msg.orientation.x
            imu_pose_msg.pose.pose.orientation.y = imu_msg.orientation.y
            imu_pose_msg.pose.pose.orientation.z = imu_msg.orientation.z
            imu_pose_msg.pose.pose.orientation.w = imu_msg.orientation.w

            depth_msg.header.stamp = CurrTS

            self.sync_imu_publisher_.publish(imu_msg)
            self.sync_imu_pose_publisher_.publish(imu_pose_msg)
            self.sync_depth_publisher_.publish(depth_msg)


    def sync_callback_D(self, imu_msg):
        # self.get_logger().info(f"D")
        CurrTS = imu_msg.header.stamp
        if (CurrTS != self.last_imu_sync_ts_sec):  # if time is the same as full sync, ignore since we've already pub'd
            imu_msg.header.stamp = CurrTS

            imu_pose_msg = PoseWithCovarianceStamped()
            imu_pose_msg.header.stamp = CurrTS
            imu_pose_msg.header.frame_id = "odom"
            imu_pose_msg.pose.pose.orientation.x = imu_msg.orientation.x
            imu_pose_msg.pose.pose.orientation.y = imu_msg.orientation.y
            imu_pose_msg.pose.pose.orientation.z = imu_msg.orientation.z
            imu_pose_msg.pose.pose.orientation.w = imu_msg.orientation.w

            self.sync_imu_publisher_.publish(imu_msg)
            self.sync_imu_pose_publisher_.publish(imu_pose_msg)


def main(args=None):
    rclpy.init(args=args)

    sync_filter_exact = SyncFilterExact()

    rclpy.spin(sync_filter_exact)

    sync_filter_exact.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
