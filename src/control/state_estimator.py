import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
import transforms3d
import numpy as np
from typing import Any
from queue import PriorityQueue
from dataclasses import dataclass, field

from scipy.spatial.transform import Rotation
from tauv_util.types import tl
from tauv_msgs.msg import (
    Pose as PoseMsg,
    XsensImuData as ImuMsg,
    TeledyneDvlData as DvlMsg,
    FluidDepth as DepthMsg,
)
from std_msgs.msg import Header
from geometry_msgs.msg import (
    Pose,
    Twist,
    Quaternion,
)
from nav_msgs.msg import Odometry as OdometryMsg
from tf2_ros import TransformBroadcaster

from .ekf import EKF


@dataclass(order=True)
class StampedMsg:
    time: float
    msg: Any = field(compare=False)


class StateEstimation(Node):
    def __init__(self):
        super().__init__("state_estimation")
        self._initialized = False

        self._tf_broadcaster = TransformBroadcaster(self)
        qos_profile = QoSProfile(depth=10)

        self._pose_pub = self.create_publisher(PoseMsg, "pose", qos_profile)
        self._odom_pub = self.create_publisher(OdometryMsg, "odom", qos_profile)

        self._imu_sub = self.create_subscription(
            ImuMsg, "/xsens_imu/data", self._receive_msg, qos_profile
        )
        self._dvl_sub = self.create_subscription(
            DvlMsg, "/teledyne_dvl/data", self._receive_msg, qos_profile
        )
        self._depth_sub = self.create_subscription(
            DepthMsg, "depth", self._receive_msg, qos_profile
        )

        self._dt = Duration(
            seconds=1.0
            / self.get_parameter("~frequency").get_parameter_value().double_value
        )
        self._horizon_delay = Duration(
            seconds=self.get_parameter("~horizon_delay")
            .get_parameter_value()
            .double_value
        )

        dvl_offset = np.array(
            self.get_parameter("~dvl_offset").get_parameter_value().double_array_value
        )
        process_covariance = np.array(
            self.get_parameter("~process_covariance")
            .get_parameter_value()
            .double_array_value
        )

        self._imu_covariance = np.array(
            self.get_parameter("~imu_covariance")
            .get_parameter_value()
            .double_array_value
        )
        self._dvl_covariance = np.array(
            self.get_parameter("~dvl_covariance")
            .get_parameter_value()
            .double_array_value
        )
        self._depth_covariance = np.array(
            self.get_parameter("~depth_covariance")
            .get_parameter_value()
            .double_array_value
        )

        self._ekf = EKF(dvl_offset, process_covariance)
        self._msg_queue = PriorityQueue()

        current_time = self.get_clock().now()
        self._last_horizon_time = (
            current_time - self._horizon_delay
            if current_time.nanoseconds > self._horizon_delay.nanoseconds
            else current_time
        )

        self._initialized = True
        self._timer = self.create_timer(self._dt.nanoseconds / 1e9, self._update)

    def _update(self):
        if not self._initialized:
            return

        current_time = self.get_clock().now()
        if current_time.nanoseconds < self._horizon_delay.nanoseconds:
            return

        horizon_time = current_time - self._horizon_delay
        while not self._msg_queue.empty():
            stamped_msg = self._msg_queue.get()
            if stamped_msg.time < horizon_time.nanoseconds:
                msg = stamped_msg.msg
                if isinstance(msg, ImuMsg):
                    self._handle_imu(msg)
                elif isinstance(msg, DvlMsg):
                    self._handle_dvl(msg)
                elif isinstance(msg, DepthMsg):
                    self._handle_depth(msg)
            else:
                self._msg_queue.put(stamped_msg)
                break
        self._publish_state(current_time)

        self._last_horizon_time = horizon_time

    def _handle_imu(self, msg: ImuMsg):
        if not self._initialized:
            return

        timestamp = self.get_clock().now()

        orientation = tl(msg.orientation)
        free_acceleration = tl(msg.free_acceleration)

        R = Rotation.from_euler("ZYX", np.flip(orientation)).inv()
        linear_acceleration = R.apply(free_acceleration)

        covariance = self._imu_covariance

        self._ekf.handle_imu_measurement(
            orientation, linear_acceleration, covariance, timestamp
        )

    def _handle_dvl(self, msg: DvlMsg):
        if not self._initialized:
            return

        timestamp = self.get_clock().now()

        if not msg.is_hr_velocity_valid:
            return

        velocity = tl(msg.hr_velocity)
        beam_std_dev = sum(msg.beam_standard_deviations) / 4.0

        covariance = self._dvl_covariance * np.array(
            [beam_std_dev, beam_std_dev, beam_std_dev]
        )

        self._ekf.handle_dvl_measurement(velocity, covariance, timestamp)

    def _handle_depth(self, msg: DepthMsg):
        if not self._initialized:
            return

        timestamp = self.get_clock().now()

        depth = np.array([msg.depth])
        covariance = self._depth_covariance

        self._ekf.handle_depth_measurement(depth, covariance, timestamp)

    def _publish_state(self, time):
        try:
            position, velocity, acceleration, orientation, angular_velocity = (
                self._ekf.get_state(time)
            )
        except ValueError as e:
            return

        # Create PoseMsg and set header
        pose_msg = PoseMsg()
        pose_msg.header = Header()
        pose_msg.header.stamp = time.to_msg()
        pose_msg.position = position
        pose_msg.velocity = velocity
        pose_msg.acceleration = acceleration
        pose_msg.orientation = orientation
        pose_msg.angular_velocity = angular_velocity

        # Publish pose message
        self._pose_pub.publish(pose_msg)

        orientation_quat = transforms3d.euler.euler2quat(*orientation)

        # Create and set up the OdometryMsg
        odom_msg = OdometryMsg()
        odom_msg.header = Header()
        odom_msg.header.stamp = time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "vehicle"
        odom_msg.pose.pose = Pose(
            position=position,
            orientation=Quaternion(
                x=orientation_quat[0],
                y=orientation_quat[1],
                z=orientation_quat[2],
                w=orientation_quat[3],
            ),
        )
        odom_msg.twist.twist = Twist(linear=velocity, angular=angular_velocity)

        # Publish odometry message
        self._odom_pub.publish(odom_msg)

        # Send transform
        self._tf_broadcaster.sendTransform(
            translation=(position.x, position.y, position.z),
            rotation=orientation_quat,
            time=time.to_msg(),
            child_frame_id="vehicle",
            parent_frame_id="odom",
        )

    def _send_odom_transform(self):
        pos = self._odom_world_pose.position
        rot = self._odom_world_pose.orientation
        self._tf_broadcaster.sendTransform(
            translation=(pos.x, pos.y, pos.z),
            rotation=(rot.x, rot.y, rot.z, rot.w),
            time=self.get_clock().now().to_msg(),
            child_frame_id="vehicle",
            parent_frame_id="odom",
        )


def main(args=None):
    rclpy.init(args=args)
    state_estimation = StateEstimation()
    rclpy.spin(state_estimation)
    state_estimation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
