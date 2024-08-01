import rclpy
import ms5837

from rclpy.node import Node

# from std_msgs.msg import Float64
from msgs.msg import DepthData as Depth
from geometry_msgs.msg import PoseStamped

MODE = 0  # MODE 0 is freshwater, MODE 1 is saltwater

sensor = ms5837.MS5837(model=ms5837.MODEL_02BA, bus=7)  # Bar02, I2C bus 7


class DepthPublisher(Node):

    def __init__(self):
        super().__init__("depth_publisher")
        self.publisher_ = self.create_publisher(Depth, "/depth/data", 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, "/depth/pose", 10)
        timer_period = 0.03  # seconds, determined empirically...
        self.timer = self.create_timer(timer_period, self.timer_callback)

        if not sensor.init():
            self.get_logger().error(f"Could not initialize depth sensor!")
            sys.exit(1)  # Exit the node with an error code

        if MODE == 1:  # default is already freshwater
            sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
            self.get_logger().info(f"Depth sensor is running with saltwater mode")
        else:
            self.get_logger().info(f"Depth sensor is running with freshwater mode")

        if not sensor.read():
            self.get_logger().error(f"Could not read depth sensor!")
            sys.exit(1)  # Exit the node with an error code

        pressure_data = sensor.pressure(ms5837.UNITS_psi)
        depth_data = sensor.depth()
        temp_data = sensor.temperature()

        self.get_logger().info(
            f"Initial Pressure: {pressure_data:.2f} m, Depth: {depth_data:.2f} m, Temperature: {temp_data:.2f} degC"
        )
        self.get_logger().info(f"Starting measurement...")

    def timer_callback(self):
        depth_msg = Depth()

        if sensor.read():
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "depth_link"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = 0.0
            pose_stamped.pose.position.y = 0.0
            pose_stamped.pose.position.z = sensor.depth()

            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.pressure = sensor.pressure(ms5837.UNITS_psi)
            depth_msg.depth = sensor.depth()
            depth_msg.temperature = sensor.temperature()

            self.publisher_.publish(depth_msg)
            self.pose_publisher_.publish(pose_stamped)

            # self.get_logger().info(f'Publishing Pressure: {pressure_msg.data:.2f} m, Depth: {depth_msg.data:.2f} m, Temperature: {temp_msg.data:.2f} degC')
        else:
            self.get_logger().info("Sensor read fail!")


def main(args=None):
    rclpy.init(args=args)

    depth_publisher = DepthPublisher()

    rclpy.spin(depth_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
