import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion
from msgs.msg import State  # Ensure this matches the actual import path for your custom message
import numpy as np
import time

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(State, 'state_topic', 10)
        self.timer = self.create_timer(0.1, self.publish_state)
        self.start_time = self.get_clock().now().to_msg()
        self.duration = 10.0  # Duration to reach the target position and orientation
        self.target_position = np.array([1.0, 0.0, 0.0])
        self.target_orientation = np.array([0.0, 0.0, 1.57])  # Radians (90 degrees)
        self.initial_position = np.array([0.0, 0.0, 0.0])
        self.initial_orientation = np.array([0.0, 0.0, 0.0])

    def publish_state(self):
        now = self.get_clock().now().to_msg()
        elapsed_time = (self.get_clock().now() - rclpy.time.Time.from_msg(self.start_time)).nanoseconds / 1e9
        
        if elapsed_time > self.duration:
            position = self.target_position
            orientation = self.target_orientation
        else:
            t = elapsed_time / self.duration
            position = (1 - t) * self.initial_position + t * self.target_position
            orientation = (1 - t) * self.initial_orientation + t * self.target_orientation

        msg = State()
        msg.header = Header()
        msg.header.stamp = now
        
        msg.position = Vector3(x=position[0], y=position[1], z=position[2])
        msg.orientation_euler = Vector3(x=orientation[0], y=orientation[1], z=orientation[2])
        
        # Set everything else to zero
        msg.linear_velocity = Vector3()
        msg.linear_acceleration = Vector3()
        msg.orientation = Quaternion()
        msg.angular_velocity = Quaternion()
        msg.angular_acceleration = Quaternion()
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    state_publisher = StatePublisher()
    rclpy.spin(state_publisher)
    
    state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

