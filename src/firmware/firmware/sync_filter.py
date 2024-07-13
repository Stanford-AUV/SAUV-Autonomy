import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from msgs.msg import (
    SensorSync as SyncMsg, 
    MTi200Data as Imu,
    DepthData as Depth 
)
from geometry_msgs.msg import TwistStamped, Twist
from message_filters import Subscriber, ApproximateTimeSynchronizer
from scipy.spatial.transform import Rotation as R

ACCEPTABLE_SLOP = 0.01  # message arrival tolerance (s) -- NOTE: the IMU is running at 100 Hz. 

class SyncFilter(Node):

    def __init__(self):
        super().__init__('sync_filter')

        # DVL Transformation: Flip x and y axes, then flip the x-axis
        self.R_dvl = np.array([[0, 1, 0],
                               [1, 0, 0],
                               [0, 0, 1]])
                               
        self.R_flip_x = np.array([[-1, 0, 0],
                                  [0, 1, 0],
                                  [0, 0, 1]])

        # IMU Transformation: Rotate 180 degrees about the z-axis, then 180 degrees about the x-axis
        self.R_imu = np.array([[-1, 0, 0],
                               [0, 1, 0],
                               [0, 0, -1]])

        self.sync_publisher_ = self.create_publisher(SyncMsg, '/IDD_synchronized_data', 10)
        self.last_imu_sync_ts_sec = math.nan

        self.dvl_sub = Subscriber(self, TwistStamped, '/dvl/twist')
        self.imu_sub = Subscriber(self, Imu, '/imu_synchronized_data')
        self.depth_sub = Subscriber(self, Depth, '/depth/data')

        self.imu_only_sub = self.create_subscription(
            Imu,
            '/imu_synchronized_data',
            self.sync_callback_D,
            10
        )

        # synchronize DVL, IMU, Depth data
        self.ts_caseA = ApproximateTimeSynchronizer( # case A: all messages available
            [self.dvl_sub, self.imu_sub, self.depth_sub], 
            queue_size=10, 
            slop=ACCEPTABLE_SLOP
        )
        self.ts_caseA.registerCallback(self.sync_callback_A)

        self.ts_caseB = ApproximateTimeSynchronizer( # case B: IMU DVL available
            [self.dvl_sub, self.imu_sub], 
            queue_size=10, 
            slop=ACCEPTABLE_SLOP
        )
        self.ts_caseB.registerCallback(self.sync_callback_B)

        self.ts_caseC = ApproximateTimeSynchronizer( # case C: IMU Depth available
            [self.imu_sub, self.depth_sub], 
            queue_size=10, 
            slop=ACCEPTABLE_SLOP
        )
        self.ts_caseC.registerCallback(self.sync_callback_C)

        self.get_logger().info(f'Running IMU/DVL/Depth data synchronization')

    def sync_callback_A(self, dvl_msg, imu_msg, depth_msg):
        self.last_imu_sync_ts_sec = imu_msg.header.stamp  # update most recent IMU time for valid full sync

        sync_msg = SyncMsg()
        sync_msg.header.stamp = self.last_imu_sync_ts_sec

        # Transform DVL data
        sync_msg.dvl_available = Bool(data=True)
        dvl_vector = np.array([dvl_msg.twist.linear.x, dvl_msg.twist.linear.y, dvl_msg.twist.linear.z])
        dvl_vector = np.dot(self.R_dvl, dvl_vector)
        dvl_vector = np.dot(self.R_flip_x, dvl_vector)
        sync_msg.dvl_data.linear.x = dvl_vector[0]
        sync_msg.dvl_data.linear.y = dvl_vector[1]
        sync_msg.dvl_data.linear.z = dvl_vector[2]

        # Transform IMU data (accelerometer)
        sync_msg.imu_available = Bool(data=True)
        imu_accel = np.array([imu_msg.free_acceleration.x, imu_msg.free_acceleration.y, imu_msg.free_acceleration.z])
        transformed_accel = np.dot(self.R_imu, imu_accel)
        imu_msg.free_acceleration.x = transformed_accel[0]
        imu_msg.free_acceleration.y = transformed_accel[1]
        imu_msg.free_acceleration.z = transformed_accel[2]

        # Transform IMU data (gyroscope)
        imu_gyro = np.array([imu_msg.rate_of_turn.x, imu_msg.rate_of_turn.y, imu_msg.rate_of_turn.z])
        transformed_gyro = np.dot(self.R_imu, imu_gyro)
        imu_msg.rate_of_turn.x = transformed_gyro[0]
        imu_msg.rate_of_turn.y = transformed_gyro[1]
        imu_msg.rate_of_turn.z = transformed_gyro[2]

        # Transform IMU orientation (Euler angles to quaternion)
        imu_orientation = R.from_euler('xyz', [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z])
        transformed_orientation = R.from_matrix(self.R_imu) * imu_orientation
        transformed_euler = transformed_orientation.as_euler('xyz')
        imu_msg.orientation.x = transformed_euler[0]
        imu_msg.orientation.y = transformed_euler[1]
        imu_msg.orientation.z = transformed_euler[2]

        sync_msg.imu_data = imu_msg

        # Add depth data
        sync_msg.depth_available = Bool(data=True)
        sync_msg.depth_data = depth_msg.depth
        
        self.sync_publisher_.publish(sync_msg)

    def sync_callback_B(self, dvl_msg, imu_msg):
        CurrTS = imu_msg.header.stamp 
        if CurrTS != self.last_imu_sync_ts_sec:  # if time is the same as full sync, ignore since we've already pub'd
            sync_msg = SyncMsg()
            sync_msg.header.stamp = CurrTS
            
            # Transform DVL data
            sync_msg.dvl_available = Bool(data=True)
            dvl_vector = np.array([dvl_msg.twist.linear.x, dvl_msg.twist.linear.y, dvl_msg.twist.linear.z])
            dvl_vector = np.dot(self.R_dvl, dvl_vector)
            dvl_vector = np.dot(self.R_flip_x, dvl_vector)
            sync_msg.dvl_data.linear.x = dvl_vector[0]
            sync_msg.dvl_data.linear.y = dvl_vector[1]
            sync_msg.dvl_data.linear.z = dvl_vector[2]

            # Transform IMU data (accelerometer)
            sync_msg.imu_available = Bool(data=True)
            imu_accel = np.array([imu_msg.free_acceleration.x, imu_msg.free_acceleration.y, imu_msg.free_acceleration.z])
            transformed_accel = np.dot(self.R_imu, imu_accel)
            imu_msg.free_acceleration.x = transformed_accel[0]
            imu_msg.free_acceleration.y = transformed_accel[1]
            imu_msg.free_acceleration.z = transformed_accel[2]

            # Transform IMU data (gyroscope)
            imu_gyro = np.array([imu_msg.rate_of_turn.x, imu_msg.rate_of_turn.y, imu_msg.rate_of_turn.z])
            transformed_gyro = np.dot(self.R_imu, imu_gyro)
            imu_msg.rate_of_turn.x = transformed_gyro[0]
            imu_msg.rate_of_turn.y = transformed_gyro[1]
            imu_msg.rate_of_turn.z = transformed_gyro[2]

            # Transform IMU orientation (Euler angles to quaternion)
            imu_orientation = R.from_euler('xyz', [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z])
            transformed_orientation = R.from_matrix(self.R_imu) * imu_orientation
            transformed_euler = transformed_orientation.as_euler('xyz')
            imu_msg.orientation.x = transformed_euler[0]
            imu_msg.orientation.y = transformed_euler[1]
            imu_msg.orientation.z = transformed_euler[2]

            sync_msg.imu_data = imu_msg

            sync_msg.depth_available = Bool(data=False)
            sync_msg.depth_data = math.nan
            
            self.sync_publisher_.publish(sync_msg)

    def sync_callback_C(self, imu_msg, depth_msg):
        CurrTS = imu_msg.header.stamp
        if CurrTS != self.last_imu_sync_ts_sec:  # if time is the same as full sync, ignore since we've already pub'd
            sync_msg = SyncMsg()
            sync_msg.header.stamp = CurrTS

            sync_msg.dvl_available = Bool(data=False)

            dvl_msg = Twist()
            dvl_msg.linear.x = math.nan
            dvl_msg.linear.y = math.nan
            dvl_msg.linear.z = math.nan
            dvl_msg.angular.x = math.nan
            dvl_msg.angular.y = math.nan
            dvl_msg.angular.z = math.nan
            sync_msg.dvl_data = dvl_msg

            # Transform IMU data (accelerometer)
            sync_msg.imu_available = Bool(data=True)
            imu_accel = np.array([imu_msg.free_acceleration.x, imu_msg.free_acceleration.y, imu_msg.free_acceleration.z])
            transformed_accel = np.dot(self.R_imu, imu_accel)
            imu_msg.free_acceleration.x = transformed_accel[0]
            imu_msg.free_acceleration.y = transformed_accel[1]
            imu_msg.free_acceleration.z = transformed_accel[2]

            # Transform IMU data (gyroscope)
            imu_gyro = np.array([imu_msg.rate_of_turn.x, imu_msg.rate_of_turn.y, imu_msg.rate_of_turn.z])
            transformed_gyro = np.dot(self.R_imu, imu_gyro)
            imu_msg.rate_of_turn.x = transformed_gyro[0]
            imu_msg.rate_of_turn.y = transformed_gyro[1]
            imu_msg.rate_of_turn.z = transformed_gyro[2]

            # Transform IMU orientation (Euler angles to quaternion)
            imu_orientation = R.from_euler('xyz', [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z])
            transformed_orientation = R.from_matrix(self.R_imu) * imu_orientation
            transformed_euler = transformed_orientation.as_euler('xyz')
            imu_msg.orientation.x = transformed_euler[0]
            imu_msg.orientation.y = transformed_euler[1]
            imu_msg.orientation.z = transformed_euler[2]

            sync_msg.imu_data = imu_msg

            sync_msg.depth_available = Bool(data=True)
            sync_msg.depth_data = depth_msg.depth
            
            self.sync_publisher_.publish(sync_msg)

    def sync_callback_D(self, imu_msg):
        CurrTS = imu_msg.header.stamp
        if CurrTS != self.last_imu_sync_ts_sec:  # if time is the same as full sync, ignore since we've already pub'd
            sync_msg = SyncMsg()
            sync_msg.header.stamp = CurrTS

            sync_msg.dvl_available = Bool(data=False)

            dvl_msg = Twist()
            dvl_msg.linear.x = math.nan
            dvl_msg.linear.y = math.nan
            dvl_msg.linear.z = math.nan
            dvl_msg.angular.x = math.nan
            dvl_msg.angular.y = math.nan
            dvl_msg.angular.z = math.nan
            sync_msg.dvl_data = dvl_msg

            # Transform IMU data (accelerometer)
            sync_msg.imu_available = Bool(data=True)
            imu_accel = np.array([imu_msg.free_acceleration.x, imu_msg.free_acceleration.y, imu_msg.free_acceleration.z])
            transformed_accel = np.dot(self.R_imu, imu_accel)
            imu_msg.free_acceleration.x = transformed_accel[0]
            imu_msg.free_acceleration.y = transformed_accel[1]
            imu_msg.free_acceleration.z = transformed_accel[2]

            # Transform IMU data (gyroscope)
            imu_gyro = np.array([imu_msg.rate_of_turn.x, imu_msg.rate_of_turn.y, imu_msg.rate_of_turn.z])
            transformed_gyro = np.dot(self.R_imu, imu_gyro)
            imu_msg.rate_of_turn.x = transformed_gyro[0]
            imu_msg.rate_of_turn.y = transformed_gyro[1]
            imu_msg.rate_of_turn.z = transformed_gyro[2]

            # Transform IMU orientation (Euler angles to quaternion)
            imu_orientation = R.from_euler('xyz', [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z])
            transformed_orientation = R.from_matrix(self.R_imu) * imu_orientation
            transformed_euler = transformed_orientation.as_euler('xyz')
            imu_msg.orientation.x = transformed_euler[0]
            imu_msg.orientation.y = transformed_euler[1]
            imu_msg.orientation.z = transformed_euler[2]

            sync_msg.imu_data = imu_msg

            sync_msg.depth_available = Bool(data=False)
            sync_msg.depth_data = math.nan
            
            self.sync_publisher_.publish(sync_msg)

def main(args=None):
    rclpy.init(args=args)

    sync_filter = SyncFilter()

    rclpy.spin(sync_filter)

    sync_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
