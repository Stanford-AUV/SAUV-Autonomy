import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
import numpy as np
from typing import Any
from queue import PriorityQueue
from dataclasses import dataclass, field

from scipy.spatial.transform import Rotation
from control.utils import to_np as tl
from msgs.msg import (
    State as StateMsg,
    # TeledyneDvlData as DvlMsg,
    # FluidDepth as DepthMsg,
)
from sensor_msgs.msg import Imu as ImuMsg
from ros_gz_interfaces.msg import Altimeter as DepthMsg
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from rclpy.time import Time

from .ekf import EKF


@dataclass(order=True)
class StampedMsg:
    time: float
    msg: Any = field(compare=False)


class StateEstimation(Node):
    def __init__(self):
        super().__init__("state_estimator")

        self._initialized = False

        self._tf_broadcaster = TransformBroadcaster(self)
        qos_profile = QoSProfile(depth=10)

        self._state_pub = self.create_publisher(StateMsg, "state", qos_profile)

        self._imu_sub = self.create_subscription(
            ImuMsg, "imu", self._receive_msg, qos_profile
        )
        # self._dvl_sub = self.create_subscription(
        #     DvlMsg, "/teledyne_dvl/data", self._receive_msg, qos_profile
        # )
        self._depth_sub = self.create_subscription(
            DepthMsg, "depth", self._receive_msg, qos_profile
        )

        self._horizon_delay = Duration(seconds=0.4)  # TODO: Don't hardcode this

        dvl_offset = np.array([-0.16256, 0, 0.110236])  # TODO: Don't hardcode this
        process_covariance = np.array(
            [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        )  # TODO: Don't hardcode this

        self._imu_covariance = np.array([0, 0, 0, 0, 0, 0])  # TODO: Don't hardcode this
        self._dvl_covariance = np.array([0, 0, 0])  # TODO: Don't hardcode this
        self._depth_covariance = np.array([0])  # TODO: Don't hardcode this

        self._ekf = EKF(dvl_offset, process_covariance)
        self._msg_queue = PriorityQueue()

        current_time = self.get_clock().now()
        self._last_horizon_time = (
            current_time - self._horizon_delay
            if current_time.nanoseconds > self._horizon_delay.nanoseconds
            else current_time
        )

        self._initialized = True
        timer_period = 0.01  # seconds (100 Hz or 10 ms update cycle)
        self.timer = self.create_timer(timer_period, self._update)

    def _receive_msg(self, msg: ImuMsg):
        if not self._initialized:
            return

        if msg.header.stamp._sec < self._last_horizon_time.seconds_nanoseconds()[0]:
            return

        stamped_msg = StampedMsg(msg.header.stamp._sec, msg)
        self._msg_queue.put(stamped_msg)

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
                # elif isinstance(msg, DvlMsg):
                #     self._handle_dvl(msg)
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

        timestamp = Time(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec,
            clock_type=self.get_clock().clock_type,
        )

        orientation = Rotation.from_quat(tl(msg.orientation)).as_euler("XYZ")
        linear_acceleration = tl(msg.linear_acceleration)

        covariance = self._imu_covariance

        self._ekf.handle_imu_measurement(
            orientation, linear_acceleration, covariance, timestamp
        )

    def _handle_dvl(self, msg):
        if not self._initialized:
            return

        timestamp = Time(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec,
            clock_type=self.get_clock().clock_type,
        )

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

        timestamp = Time(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec,
            clock_type=self.get_clock().clock_type,
        )

        depth = np.array([msg.vertical_reference + msg.vertical_position])
        covariance = self._depth_covariance

        self._ekf.handle_depth_measurement(depth, covariance, timestamp)

    def _publish_state(self, time):
        try:
            position, velocity, acceleration, orientation, angular_velocity = (
                self._ekf.get_state(time)
            )
        except ValueError as e:
            return

        # Create StateMsg and set header
        state_msg = StateMsg()
        state_msg.header = Header()
        state_msg.header.stamp = time.to_msg()
        state_msg.position = position
        state_msg.linear_velocity = velocity
        state_msg.linear_acceleration = acceleration
        state_msg.orientation = orientation
        state_msg.euler_velocity = angular_velocity
        # TODO: state_msg.euler_acceleration

        # Publish state message
        self._state_pub.publish(state_msg)

        self.get_logger().info(
            f"Estimated position: {state_msg.position.x:.2f} {state_msg.position.y:.2f} {state_msg.position.z:.2f}"
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
