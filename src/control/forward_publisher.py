import rclpy
from rclpy.node import Node
from msgs.msg import Wrench
from geometry_msgs.msg import Vector3
import numpy as np
import sys
import time
import pygame

from std_msgs.msg import String


class ForwardPublisher(Node):

    def __init__(self):
        super().__init__('forward_publisher')
        self.publisher_ = self.create_publisher(Wrench, 'desired_wrench', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        pygame.init()

    def timer_callback(self):
        wrench = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    wrench += np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
                elif event.key == pygame.K_DOWN:
                    wrench += np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
                elif event.key == pygame.K_LEFT:
                    wrench += np.array([0.0, -1.0, 0.0, 0.0, 0.0, 0.0]) 
                elif event.key == pygame.K_RIGHT:
                    wrench += np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0]) 
                elif event.key == pygame.K_u:
                    wrench += np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]) 
                elif event.key == pygame.K_j:
                    wrench += np.array([0.0, 0.0, -1.0, 0.0, 0.0, 0.0]) 
        msg = Wrench()
        msg.force = Vector3(x=wrench[0], y=wrench[1], z=wrench[2])
        msg.torque = Vector3(x=wrench[3], y=wrench[4], z=wrench[5])
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {wrench}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    forward_publisher = ForwardPublisher()

    rclpy.spin(forward_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    forward_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
