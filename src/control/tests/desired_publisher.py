import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class DesiredPublisher(Node) :

    def __init__(self) :
        super().__init__('desired_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'desired',
            10)
        
        timer_period = 10 # seconds
        self.timer = self.create_timer(timer_period, self.publish_output)

    def publish_output(self) :
            msg = Float64MultiArray()
            msg.data = [[1,1,1,1,1,1]]
            self.publisher_publish(msg)
            self.get_logger().info('Publishing output: %s' % msg.data)

def main(args = None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
     main()