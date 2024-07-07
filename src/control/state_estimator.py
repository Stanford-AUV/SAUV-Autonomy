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
from msgs.msg import State as StateMsg
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from rclpy.time import Time

from msgs.msg import SensorSync, MTi200Data
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

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

        self._sensor_sync_sub = self.create_subscription(
            SensorSync, "/IDD_synchronized_data", self._receive_msg, qos_profile
        )

        self._dt = Duration(seconds=1.0)  # TODO: Don't hardcode this
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

        # Initialization parameters in StateEstimation class
        # NOTE: orientation covariance: 0.2 deg RMS --> 1.218e-5 
        self._imu_covariance = np.array([1.218e-5 , 1.218e-5 ,1.218e-5 , 1.54545378e-04,   1.63320184e-04, 3.22157790e-05])  # Adjust based on your IMU specifications
        self._dvl_covariance = np.array([0.1, 0.1, 0.1]) 
        self._depth_covariance = np.array([7.196207822120195e-06])  # Adjust based on your depth sensor specifications


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

    def _receive_msg(self, msg: SensorSync):
        if not self._initialized:
            return

        if msg.header.stamp.sec < self._last_horizon_time.seconds_nanoseconds()[0]:
            return

        stamped_msg = StampedMsg(msg.header.stamp.sec, msg)
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
                if msg.imu_available.data:
                    self._handle_imu(msg.imu_data)
                if msg.dvl_available.data:
                    self._handle_dvl(msg.dvl_data)
                if msg.depth_available.data:
                    self._handle_depth(msg.depth_data)
            else:
                self._msg_queue.put(stamped_msg)
                break
        self._publish_state(current_time)

        self._last_horizon_time = horizon_time

    def _handle_imu(self, imu_data: MTi200Data):
        if not self._initialized:
            return

        timestamp = Time(
            seconds=imu_data.header.stamp.sec,
            nanoseconds=imu_data.header.stamp.nanosec,
            clock_type=self.get_clock().clock_type,
        )

        # Extract orientation directly as Euler angles
        orientation = np.array([
            imu_data.orientation.x,
            imu_data.orientation.y,
            imu_data.orientation.z
        ])

        linear_acceleration = np.array([
            imu_data.free_acceleration.x,
            imu_data.free_acceleration.y,
            imu_data.free_acceleration.z
        ])

        covariance = self._imu_covariance

        self._ekf.handle_imu_measurement(
            orientation, linear_acceleration, covariance, timestamp
        )

    def _handle_dvl(self, dvl_data: Twist):
        if not self._initialized:
            return

        timestamp = self.get_clock().now()  # Using current time, adjust if needed

        velocity = np.array([
            dvl_data.linear.x,
            dvl_data.linear.y,
            dvl_data.linear.z
        ])
        covariance = self._dvl_covariance

        self._ekf.handle_dvl_measurement(velocity, covariance, timestamp)

    def _handle_depth(self, depth_data: float):
        if not self._initialized:
            return

        timestamp = self.get_clock().now()  # Using current time, adjust if needed

        depth = np.array([depth_data])
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

        orientation_array = np.array([orientation.x, orientation.y, orientation.z])
        quaternion = Rotation.from_euler('xyz', orientation_array).as_quat()
        state_msg.orientation.x = quaternion[0]
        state_msg.orientation.y = quaternion[1]
        state_msg.orientation.z = quaternion[2]
        state_msg.orientation.w = quaternion[3]

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
