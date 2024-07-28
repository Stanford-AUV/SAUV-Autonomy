import rclpy
from rclpy.node import Node
from msgs.msg import MTi200Data as Imu
import numpy as np


class IMUNoiseEstimator(Node):
    def __init__(self):
        super().__init__("imu_noise_estimator")
        self.subscription = self.create_subscription(
            Imu, "/imu_synchronized_data", self.imu_callback, 10
        )
        self.subscription  # prevent unused variable warning

        self.imu_data_list = []
        self.get_logger().info("IMU Noise Estimator Node has been started")

    def imu_callback(self, msg):
        # Extract relevant data
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z]
        linear_acceleration = [
            msg.free_acceleration.x,
            msg.free_acceleration.y,
            msg.free_acceleration.z,
        ]
        angular_velocity = [msg.rate_of_turn.x, msg.rate_of_turn.y, msg.rate_of_turn.z]

        # Store the data
        self.imu_data_list.append((orientation, linear_acceleration, angular_velocity))

        # For demonstration purposes, let's collect data for a few seconds
        if len(self.imu_data_list) >= 10000:  # Assuming a high frequency IMU
            self.compute_statistics()
            self.imu_data_list = []  # Reset the list for new collection

    def compute_statistics(self):
        imu_data_array = np.array(self.imu_data_list)
        accelerations = np.array(
            [data[1] for data in imu_data_array]
        )  # Extract linear accelerations
        angular_velocities = np.array(
            [data[2] for data in imu_data_array]
        )  # Extract angular velocities

        # Calculate mean and covariance
        accel_mean = np.mean(accelerations, axis=0)
        gyro_mean = np.mean(angular_velocities, axis=0)

        accel_cov = np.cov(accelerations, rowvar=False)
        gyro_cov = np.cov(angular_velocities, rowvar=False)

        self.get_logger().info(f"Accelerometer Mean: {accel_mean}")
        self.get_logger().info(f"Gyroscope Mean: {gyro_mean}")
        self.get_logger().info(f"Accelerometer Covariance Matrix:\n{accel_cov}")
        self.get_logger().info(f"Gyroscope Covariance Matrix:\n{gyro_cov}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUNoiseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
