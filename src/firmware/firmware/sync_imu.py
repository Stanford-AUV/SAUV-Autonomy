import rclpy
import ms5837
import math

from rclpy.node import Node
from std_msgs.msg import Float64
from msgs.msg import MTi200Data as Imu
from geometry_msgs.msg import TwistStamped, QuaternionStamped, Vector3Stamped, Vector3
from sensor_msgs.msg import TimeReference
from message_filters import Subscriber, TimeSynchronizer
import tf_transformations

class ImuGrouper(Node):

    def __init__(self):
        super().__init__('imu_grouper')

        self.sample_time = Subscriber(self, TimeReference, '/imu/time_ref')
        self.twist = Subscriber(self, TwistStamped, '/filter/twist') # NOTE: currently missing- we do not have this printing for some reason!
        self.quat = Subscriber(self, QuaternionStamped, '/filter/quaternion')
        self.free_accel = Subscriber(self, Vector3Stamped, '/filter/free_acceleration')
        self.linear_accel = Subscriber(self, Vector3Stamped, '/imu/acceleration')

        self.publisher_ = self.create_publisher(Imu, '/imu_synchronized_data', 10)
        
        # TimeSynchronizer
        self.ts = TimeSynchronizer([self.sample_time, self.quat, self.free_accel, self.linear_accel], 10)
        self.ts.registerCallback(self.callback)

        self.get_logger().info(f'Running IMU data synchronization')


    def callback(self, sample_time, quat, free_accel, linear_accel):
        
        # Bundle the messages into a single synchronized message
        imu_synced = Imu()

        # IMU sample time
        imu_synced.imu_time = sample_time

        # Extract Euler rate of turn from twist
        imu_synced.rate_of_turn.x = math.nan
        imu_synced.rate_of_turn.y = math.nan
        imu_synced.rate_of_turn.z = math.nan

        # Convert quaternion to Euler representation 
        euler = tf_transformations.euler_from_quaternion([quat.quaternion.x, quat.quaternion.y, quat.quaternion.z, quat.quaternion.w])
        imu_synced.orientation = Vector3()
        imu_synced.orientation.x = euler[0]
        imu_synced.orientation.y = euler[1]
        imu_synced.orientation.z = euler[2]

        # Free acceleration
        imu_synced.free_acceleration.x = free_accel.vector.x
        imu_synced.free_acceleration.y = free_accel.vector.y
        imu_synced.free_acceleration.z = free_accel.vector.z

        # Linear acceleration
        imu_synced.linear_acceleration.x = linear_accel.vector.x
        imu_synced.linear_acceleration.y = linear_accel.vector.y
        imu_synced.linear_acceleration.z = linear_accel.vector.z
        
        # Publish the synchronized message
        self.publisher_.publish(imu_synced)


def main(args=None):
    rclpy.init(args=args)

    imu_grouper = ImuGrouper()

    rclpy.spin(imu_grouper)

    imu_grouper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()