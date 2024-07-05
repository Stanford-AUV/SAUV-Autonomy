import rclpy
import ms5837
import math

from rclpy.node import Node
from std_msgs.msg import Float64
from msgs.msg import (
    MTi200Data as Imu 
)
from geometry_msgs.msg import TwistStamped, QuaternionStamped, Vector3Stamped
from sensor_msgs.msg import TimeReference
from message_filters import Subscriber, TimeSynchronizer
import tf_transformations

ACCEPTABLE_SLOP = 0.0 # must be effectively perfect


## UPDATE THIS!

## UPDATE THIS!

## UPDATE THIS!

## UPDATE THIS!

## UPDATE THIS!


class ImuGrouper(Node):

    def __init__(self):
        super().__init__('imu_grouper')

        self.sample_time = Subscriber(self, TimeReference, '/imu/time_ref')
        self.twist = Subscriber(self, TwistStamped, '/filter/twist')
        self.quat = Subscriber(self, QuaternionStamped, '/filter/quaternion')
        self.free_accel = Subscriber(self, Vector3Stamped, '/filter/free_acceleration')
        self.linear_accel = Subscriber(self, Vector3Stamped, '/imu/acceleration')

        self.sync_publisher_ = self.create_publisher(Imu, '/imu_synchronized_data', 10)
        
        # TimeSynchronizer
        self.ts = TimeSynchronizer([self.sample_time, self.twist, self.quat, self.free_accel, self.linear_accel], 10)
        self.ts.registerCallback(self.callback)


    def callback(self, sample_time, twist, quat, free_accel, linear_accel):
        self.get_logger().info(f'Synchronized IMU data received')
        
        # Bundle the messages into a single synchronized message
        imu_synced = Imu()

        # IMU sample time
        imu_synced.imu_time = sample_time

        # Extract Euler rate of turn from twist
        imu_synced.rate_of_turn.x = twist.angular.x 
        imu_synced.rate_of_turn.y = twist.angular.y
        imu_synced.rate_of_turn.z = twist.angular.z 

        # Convert quaternion to Euler representation 
        imu_synced.orientation = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        # Free acceleration
        imu_synced.free_acceleration.x = free_accel.x
        imu_synced.free_acceleration.y = free_accel.y
        imu_synced.free_acceleration.z = free_accel.z

        # Linear acceleration
        imu_synced.linear_acceleration.x = linear_accel.x
        imu_synced.linear_acceleration.y = linear_accel.y
        imu_synced.linear_acceleration.z = linear_accel.z
        
        # Publish the synchronized message
        self.pub.publish(imu_synced)


def main(args=None):
    rclpy.init(args=args)

    imu_grouper = ImuGrouper()

    rclpy.spin(imu_grouper)

    imu_grouper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()