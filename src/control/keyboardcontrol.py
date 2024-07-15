import rclpy
from rclpy.node import Node
import numpy as np
import sys
import time
import getch
import threading
import signal

from msgs.msg import Wrench
from geometry_msgs.msg import Vector3

MAX_THRUST = 0.8
MAX_TORQUE = 0.1
THRUST_INCREMENT = 0.1

class KeyboardControl(Node):
    def __init__(self):
        super().__init__("keyboard_control")
        self.publisher_ = self.create_publisher(Wrench, "desired_wrench", 10)
        self.record = "--record" in sys.argv
        self.file = open("wrench.txt", "w") if self.record else None

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.wrench = np.zeros(6)
        self.current_thrust = 0.0

        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.torque_z = 0.0

        self.running = True

        self.key_thread = threading.Thread(target=self.key_listener)
        self.key_thread.daemon = True
        self.key_thread.start()

        # Register the signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.get_logger().info('Shutting down...')
        self.running = False
        self.destroy_node()
        rclpy.shutdown()

    def key_listener(self):
        while self.running:
            key = getch.getch()
            self.on_press(key)

    def on_press(self, key):
        self.get_logger().info(f"Key pressed: {key}")
        if key == 'w':
            self.force_x = self.current_thrust
        elif key == 's':
            self.force_x = -self.current_thrust
        elif key == 'd':
            self.force_y = self.current_thrust
        elif key == 'a':
            self.force_y = -self.current_thrust
        elif key == 'i':
            self.force_z = self.current_thrust
        elif key == 'k':
            self.force_z = -self.current_thrust
        elif key == 'j':
            self.torque_z = MAX_TORQUE
        elif key == 'l':
            self.torque_z = -MAX_TORQUE
        elif key == 't':
            self.current_thrust = min(self.current_thrust + THRUST_INCREMENT, MAX_THRUST)
            self.get_logger().info(f"Current thrust increased to: {self.current_thrust}")
        elif key == 'g':
            self.current_thrust = max(self.current_thrust - THRUST_INCREMENT, 0.0)
            self.get_logger().info(f"Current thrust decreased to: {self.current_thrust}")
        elif key == 'x':
            self.force_x = self.force_y = self.force_z = self.torque_z = 0.0

    def timer_callback(self):
        self.wrench = np.array([self.force_x, self.force_y, self.force_z, 0.0, 0.0, self.torque_z])
        msg = Wrench()
        msg.force = Vector3(x=self.force_x, y=self.force_y, z=self.force_z)
        msg.torque = Vector3(x=0.0, y=0.0, z=self.torque_z)
        self.publisher_.publish(msg)

        self.get_logger().info(f"Publishing: {self.wrench}")

        if self.record:
            timestamp = time.time_ns()
            self.file.write(f"{timestamp}: {self.wrench}\n")
            self.file.flush()

    def destroy_node(self):
        if self.record:
            self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
