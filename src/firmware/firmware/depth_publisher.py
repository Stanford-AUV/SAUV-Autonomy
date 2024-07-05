import rclpy
import ms5837

from rclpy.node import Node
from std_msgs.msg import Float64

MODE = 0 # MODE 0 is freshwater, MODE 1 is saltwater

sensor = ms5837.MS5837(model=ms5837.MODEL_02BA, bus=7) # Bar02, I2C bus 7

class DepthPublisher(Node):

    def __init__(self):
        super().__init__('depth_publisher')
        self.pressure_publisher_ = self.create_publisher(Float64, 'depth_sensor/pressure_psi', 10)
        self.depth_publisher_ = self.create_publisher(Float64, 'depth_sensor/depth_meter', 10)
        self.temp_publisher_ = self.create_publisher(Float64, 'depth_sensor/temp_celsius', 10)
        timer_period = 0.03  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        if not sensor.init():
            self.get_logger().error(f'Could not initialize depth sensor!')
            sys.exit(1)  # Exit the node with an error code

        if MODE == 1: # default is already freshwater
            sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
            self.get_logger().info(f'Depth sensor is running with saltwater mode')
        else:
            self.get_logger().info(f'Depth sensor is running with freshwater mode')

        if not sensor.read():
            self.get_logger().error(f'Could not read depth sensor!')
            sys.exit(1)  # Exit the node with an error code

        pressure_data= sensor.pressure(ms5837.UNITS_psi)
        depth_data = sensor.depth()
        temp_data = sensor.temperature()

        self.get_logger().info(f'Initial Pressure: {pressure_data:.2f} m, Depth: {depth_data:.2f} m, Temperature: {temp_data:.2f} degC')
        self.get_logger().info(f'Starting measurement...')

    def timer_callback(self):
        pressure_msg = Float64()
        depth_msg = Float64()
        temp_msg = Float64()
        
        if sensor.read():
            pressure_msg.data = sensor.pressure(ms5837.UNITS_psi)
            depth_msg.data = sensor.depth()
            temp_msg.data = sensor.temperature()
        
            self.pressure_publisher_.publish(pressure_msg)
            self.depth_publisher_.publish(depth_msg)
            self.temp_publisher_.publish(temp_msg)

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


if __name__ == '__main__':
    main()
