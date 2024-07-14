import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
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
from geometry_msgs.msg import Vector3

from msgs.msg import SensorSync, MTi200Data
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from .ekf import EKF
from geometry_msgs.msg import Vector3

@dataclass(order=True)
class StampedMsg:
    time: float
    msg: Any = field(compare=False)

class StateEstimation(Node):
    def __init__(self):
        super().__init__("state_estimator")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('ekf.dt',0.1),
                ('ekf.horizon_delay',0.04),
                ('dvl.offset_x',-0.16256),
                ('dvl.offset_y',0.0),
                ('dvl.offset_z',0.110236),
                ('dvl.cov_x',0.0001),
                ('dvl.cov_y',0.0001),
                ('dvl.cov_z',0.0001),
                ('imu.orientation_cov_x',1.218e-5),
                ('imu.orientation_cov_y',1.218e-5),
                ('imu.orientation_cov_z',1.218e-5),
                ('imu.accel_cov_x',1.218e-5),
                ('imu.accel_cov_y',1.218e-5),
                ('imu.accel_cov_z',1.218e-5),
                ('depth.cov_z',7.196207822120195e-06),
                ('process_cov.position_x',0.01),
                ('process_cov.position_y',0.01),
                ('process_cov.position_z',0.01),
                ('process_cov.linear_vel_x',0.01),
                ('process_cov.linear_vel_y',0.01),
                ('process_cov.linear_vel_z',0.01),
                ('process_cov.linear_accel_x',0.01),
                ('process_cov.linear_accel_y',0.01),
                ('process_cov.linear_accel_z',0.01),
                ('process_cov.angular_position_x',0.01),
                ('process_cov.angular_position_y',0.01),
                ('process_cov.angular_position_z',0.01),
                ('process_cov.angular_vel_x',0.01),
                ('process_cov.angular_vel_y',0.01),
                ('process_cov.angular_vel_z',0.01)
            ]
        )

        self._initialized = False

        self._tf_broadcaster = TransformBroadcaster(self)
        qos_profile = QoSProfile(depth=10)

        self._state_pub = self.create_publisher(StateMsg, "state", qos_profile)
        self._pose_pub = self.create_publisher(PoseStamped, "visualizable_state", qos_profile)
        

        self._sensor_sync_sub = self.create_subscription(
            SensorSync, "/IDD_synchronized_data", self._receive_msg, qos_profile
        )

        self._dt = Duration(seconds=self.get_parameter('ekf.dt').value)  
        self._horizon_delay = Duration(seconds=self.get_parameter('ekf.horizon_delay').value)  

        dvl_offset = np.array([self.get_parameter('dvl.offset_x').value, self.get_parameter('dvl.offset_y').value, self.get_parameter('dvl.offset_z').value]) 
        process_covariance = np.array(
            [
                self.get_parameter('process_cov.position_x').value,
                self.get_parameter('process_cov.position_y').value,
                self.get_parameter('process_cov.position_z').value,
                self.get_parameter('process_cov.linear_vel_x').value,
                self.get_parameter('process_cov.linear_vel_y').value,
                self.get_parameter('process_cov.linear_vel_z').value,
                self.get_parameter('process_cov.linear_accel_x').value,
                self.get_parameter('process_cov.linear_accel_y').value,
                self.get_parameter('process_cov.linear_accel_z').value,
                self.get_parameter('process_cov.angular_position_x').value,
                self.get_parameter('process_cov.angular_position_y').value,
                self.get_parameter('process_cov.angular_position_z').value,
                self.get_parameter('process_cov.angular_vel_x').value,
                self.get_parameter('process_cov.angular_vel_y').value,
                self.get_parameter('process_cov.angular_vel_z').value,
            ]
        ) 

        # Initialization parameters in StateEstimation class
        # NOTE: orientation covariance: 0.2 deg RMS --> 1.218e-5 
        self._imu_covariance = np.array([self.get_parameter('imu.orientation_cov_x').value, self.get_parameter('imu.orientation_cov_y').value, self.get_parameter('imu.orientation_cov_z').value, 
                                         self.get_parameter('imu.accel_cov_x').value, self.get_parameter('imu.accel_cov_y').value, self.get_parameter('imu.accel_cov_z').value])  # Adjust based on your IMU specifications
        self._dvl_covariance = np.array([self.get_parameter('dvl.cov_x').value, self.get_parameter('dvl.cov_y').value, self.get_parameter('dvl.cov_z').value]) 
        self._depth_covariance = np.array([self.get_parameter('depth.cov_z').value])  # Adjust based on your depth sensor specifications

        self._ekf = EKF(dvl_offset, process_covariance, {
            'imu': self._imu_covariance,
            'dvl': self._dvl_covariance,
            'depth': self._depth_covariance[0]
        })
        self._msg_queue = PriorityQueue()

        current_time = self.get_clock().now()
        self._last_horizon_time = (
            current_time - self._horizon_delay
            if current_time.nanoseconds > self._horizon_delay.nanoseconds
            else current_time
        )

        self._initialized = True
        self._timer = self.create_timer(self._dt.nanoseconds / 1e9, self._update)
        self.get_logger().info("State estimator initialized")

        self.initial_time_offset = None

    def _receive_msg(self, msg: SensorSync):
        if not self._initialized:
            self.get_logger().info("State estimator not initialized")
            return

        current_time = self.get_clock().now()

        # Create a Time object for the message stamp with the same clock type as current_time
        msg_time = Time(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec,
            clock_type=current_time.clock_type
        )

        if self.initial_time_offset is None:
            self.initial_time_offset = current_time - msg_time

        adjusted_stamp = msg_time + self.initial_time_offset

        # Use seconds_nanoseconds() method to get the seconds part
        if adjusted_stamp.seconds_nanoseconds()[0] < self._last_horizon_time.seconds_nanoseconds()[0]:
            return

        stamped_msg = StampedMsg(adjusted_stamp.nanoseconds, msg)
        self._msg_queue.put(stamped_msg)

    def _update(self):
        if not self._initialized:
            self.get_logger().info("State estimator not initialized in _update")
            return

        current_time = self.get_clock().now()
        if current_time.nanoseconds < self._horizon_delay.nanoseconds:
            return

        horizon_time = current_time - self._horizon_delay
        message_processed = False

        while not self._msg_queue.empty():
            stamped_msg = self._msg_queue.get()
            if stamped_msg.time < horizon_time.nanoseconds:
                msg = stamped_msg.msg
                if msg.imu_available.data:
                    self._handle_imu(msg.imu_data)
                    message_processed = True
                if msg.dvl_available.data:
                    self._handle_dvl(msg.dvl_data)
                    message_processed = True
                if msg.depth_available.data:
                    self._handle_depth(msg.depth_data)
                    message_processed = True
            else:
                self._msg_queue.put(stamped_msg)
                break

        if message_processed:
            dt = (current_time - self._last_horizon_time).nanoseconds / 1e9
            self._ekf.predict(dt)
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

        # self.get_logger().info(
        #     f"IMU ACC: {linear_acceleration[0]:.2f} {linear_acceleration[1]:.2f} {linear_acceleration[2]:.2f}"
        # )

        covariance = self._imu_covariance

        # self.get_logger().info(f"IMU data: orientation={orientation}, linear_acceleration={linear_acceleration}")
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
        # self.get_logger().info(
        #     f"DVL VEL: {velocity[0]:.2f} {velocity[1]:.2f} {velocity[2]:.2f}"
        # )
        covariance = self._dvl_covariance

        # self.get_logger().info(f"DVL data: velocity={velocity}")
        self._ekf.handle_dvl_measurement(velocity, covariance, timestamp)

    def _handle_depth(self, depth_data: float):
        if not self._initialized:
            return

        timestamp = self.get_clock().now()  # Using current time, adjust if needed

        depth = np.array([depth_data])
        covariance = self._depth_covariance

        # self.get_logger().info(f"Depth data: depth={depth}")
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

        # Convert numpy arrays to ROS messages
        state_msg.position = Vector3(x=-position[0], y=position[1], z=position[2])
        state_msg.linear_velocity = Vector3(x=velocity[0], y=velocity[1], z=velocity[2])
        state_msg.linear_acceleration = Vector3(x=acceleration[0], y=acceleration[1], z=acceleration[2])

        state_msg.orientation_euler = Vector3(x=-orientation[0], y=-orientation[1], z=-orientation[2])

        # Convert orientation (Euler angles) to quaternion
        quaternion = Rotation.from_euler('xyz', orientation).as_quat()
        state_msg.orientation.x = -quaternion[0]
        state_msg.orientation.y = -quaternion[1]
        state_msg.orientation.z = -quaternion[2]
        state_msg.orientation.w = quaternion[3]

        # Convert angular velocity to quaternion representation
        # CURRENTLY NOT PUBLISHED!!!! MAY NEED TO BE UPDATED TO REFLECT ANGLE REFLECTION SEE ABOVE ^^
        angular_velocity_quat = Rotation.from_euler('xyz', angular_velocity).as_quat()
        state_msg.angular_velocity.x = angular_velocity_quat[0]
        state_msg.angular_velocity.y = angular_velocity_quat[1]
        state_msg.angular_velocity.z = angular_velocity_quat[2]
        state_msg.angular_velocity.w = angular_velocity_quat[3]

        # Publish state message
        self._state_pub.publish(state_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = time.to_msg()
        pose_msg.header.frame_id = 'map'  # Adjust as necessary
        pose_msg.pose.position.x = -position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        self._pose_pub.publish(pose_msg)

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
