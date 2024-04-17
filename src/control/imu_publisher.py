import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from xsens_mti_driver import MTi

class XsensMTiPublisher(Node) :
    
    def __init__(self) :
        super().init('xsens_mti_publisher')
        self._publisher = self.create_publisher(Imu, 'imu', 10)
        self.mti = MTi() # TODO: Initialize your MTI driver here

    def publish_imu_data(self) :
        imu_msg = Imu()

        # Fill in the header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_frame'

        # Fill in the orientation (Quaternion)
        # TODO: Replace these lines with data from the MTi driver
        imu_msg.orientation = Quaternion() 

        # Fill in the angular velocity
        # TODO: Replace these lines with data from the MTi driver
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        # Fill in the linear acceleration
        # TODO: Replace these lines with data from the MTi driver
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        # Publish the message
        self.publisher_.publish(imu_msg)
        