import rclpy
import ms5837

from rclpy.node import Node
from std_msgs.msg import Float64
from msgs.msg import (
    SensorSync as SyncMsg
    # TeledyneDvlData as DvlMsg,
    # FluidDepth as DepthMsg,
)

# HOW TO IMPORT MESSAGES 

sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_02BA, bus=7) # Bar02, I2C bus 7

class SyncFilter(Node):

    def __init__(self):
        super().__init__('sync_pubsub')
        self.sync_publisher_ = self.create_publisher(Float64, 'sensors/synchronized_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


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

            self.get_logger().info(f'Publishing Pressure: {pressure_msg.data:.2f} m, Depth: {depth_msg.data:.2f} m, Temperature: {temp_msg.data:.2f} degC')
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