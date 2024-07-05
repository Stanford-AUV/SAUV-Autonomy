import rclpy
import ms5837
import math

from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from msgs.msg import (
    SensorSync as SyncMsg, 
    MTi200Data as Imu,
    DepthData as Depth 
)
from geometry_msgs.msg import Twist
from message_filters import Subscriber, ApproximateTimeSynchronizer

ACCEPTABLE_SLOP = 0.000125 # 1 / 400 Hz divided by 2, this may need to be changed based on IMU frequency

class SyncFilter(Node):

    def __init__(self):
        super().__init__('sync_filter')

        self.sync_publisher_ = self.create_publisher(SyncMsg, '/IDD_synchronized_data', 10)
        self.last_imu_sync_ts_sec = math.nan

        self.dvl_sub = Subscriber(self, Twist, '/dvl/velocity')
        self.imu_sub = Subscriber(self, Imu, '/imu_synchronized_data')
        self.depth_sub = Subscriber(self, Depth, '/depth/data')

        # TODO: ALSO SUBSCRIBE TO ALL OTHER CRITICAL IMU MESSAGES

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
        self.last_imu_sync_ts_sec = imu_msg.header.stamp.sec # update most recent IMU time for valid full sync

        sync_msg = SyncMsg()
        sync_msg.dvl_available = Bool(data=True)
        sync_msg.dvl_data = dvl_msg

        sync_msg.imu_available = Bool(data=True)
        sync_msg.imu_data = imu_msg

        sync_msg.depth_available = Bool(data=True)
        sync_msg.depth_data = depth_msg
        
        self.sync_publisher_.publish(sync_msg)
        # self.get_logger().info(f'Publishing fully synchronized data: {sync_msg}')

    def sync_callback_B(self, dvl_msg, imu_msg):
        CurrTS = imu_msg.header.stamp.sec 
        if CurrTS != self.last_imu_sync_ts_sec: # if time is the same as full sync, ignore since we've already pub'd
            sync_msg = SyncMsg()
            sync_msg.dvl_available = Bool(data=True)
            sync_msg.dvl_data = dvl_msg

            sync_msg.imu_available = Bool(data=True)
            sync_msg.imu_data = imu_msg

            sync_msg.depth_available = Bool(data=False)
            sync_msg.depth_data = math.nan
            
            self.sync_publisher_.publish(sync_msg)
            # self.get_logger().info(f'Publishing partially synchronized data: {sync_msg}')

    def sync_callback_C(self, imu_msg, depth_msg):
        CurrTS = imu_msg.header.stamp.sec
        if CurrTS != self.last_imu_sync_ts_sec: # if time is the same as full sync, ignore since we've already pub'd
            sync_msg = SyncMsg()
            sync_msg.dvl_available = Bool(data=False)

            dvl_msg = Twist()
            dvl_msg.linear.x = math.nan
            dvl_msg.linear.y = math.nan
            dvl_msg.linear.z = math.nan
            dvl_msg.angular.x = math.nan
            dvl_msg.angular.y = math.nan
            dvl_msg.angular.z = math.nan
            sync_msg.dvl_data = dvl_msg

            sync_msg.imu_available = Bool(data=True)
            sync_msg.imu_data = imu_msg

            sync_msg.depth_available = Bool(data=True)
            sync_msg.depth_data = depth_msg.depth
            
            self.sync_publisher_.publish(sync_msg)
            # self.get_logger().info(f'Publishing partially synchronized data: {sync_msg}')


def main(args=None):
    rclpy.init(args=args)

    sync_filter = SyncFilter()

    rclpy.spin(sync_filter)

    sync_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()