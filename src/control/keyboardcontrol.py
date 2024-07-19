import rclpy
from rclpy.node import Node
import numpy as np
import sys
import threading
import termios
import tty
import signal
import selectors
import types

from msgs.msg import Wrench
from geometry_msgs.msg import Vector3

MAX_THRUST = 0.8
MAX_TORQUE = 0.1
THRUST_INCREMENT = 0.1

class KeyboardControl(Node):
    def __init__(self):
        super().__init__("keyboard_control")
        self.publisher_ = self.create_publisher(Wrench, "desired_wrench", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.wrench = np.zeros(6)
        self.current_thrust = 0.0

        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.torque_z = 0.0

        self.running = True
        self.shutdown_complete = False

        self.key_state = {
            'w': False, 's': False, 'a': False, 'd': False,
            'i': False, 'k': False, 'j': False, 'l': False
        }

        self.get_logger().info("Ready to read keyboard input.")
        self.get_logger().info("- 'x' to kill ALL thrust")
        self.get_logger().info("- 't g' to increase/decrease thrust magnitude (without moving)")
        self.get_logger().info("- 'w a s d' to set planar direction")
        self.get_logger().info("- 'i' to set upward vertical thrust")
        self.get_logger().info("- 'k' to set downward vertical thrust")
        self.get_logger().info("- 'j l' for rotation")

        self.selector = selectors.DefaultSelector()
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        self.selector.register(self.fd, selectors.EVENT_READ, self.key_event)

        self.key_thread = threading.Thread(target=self.key_listener)
        self.key_thread.start()

        # Register the signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, sig, frame):
        if not self.shutdown_complete:
            self.get_logger().info('Shutting down...')
            self.running = False
            self.selector.close()
            self.key_thread.join()
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            self.destroy_node()
            rclpy.shutdown()
            self.shutdown_complete = True

    def key_listener(self):
        while self.running:
            events = self.selector.select(timeout=1)
            for key, mask in events:
                callback = key.data
                callback()

    def key_event(self):
        key = sys.stdin.read(1)
        if key:
            self.on_press(key)
        if key == '\x03':  # Detect Ctrl+C to trigger signal handler
            self.signal_handler(signal.SIGINT, None)
    
    def on_press(self, key):
        self.get_logger().info(f"Key pressed: {key}")
        if key in self.key_state:
            self.key_state[key] = True
        if key == 't':
            self.current_thrust = min(self.current_thrust + THRUST_INCREMENT, MAX_THRUST)
            self.get_logger().info(f"Current thrust increased to: {self.current_thrust}")
        elif key == 'g':
            self.current_thrust = max(self.current_thrust - THRUST_INCREMENT, 0.0)
            self.get_logger().info(f"Current thrust decreased to: {self.current_thrust}")
        elif key == 'x':
            self.force_x = self.force_y = self.force_z = self.torque_z = 0.0
        self.update_forces()

    def on_release(self, key):
        self.get_logger().info(f"Key released: {key}")
        if key in self.key_state:
            self.key_state[key] = False
        self.update_forces()

    def update_forces(self):
        self.force_x = self.current_thrust if self.key_state['w'] else -self.current_thrust if self.key_state['s'] else 0.0
        self.force_y = self.current_thrust if self.key_state['d'] else -self.current_thrust if self.key_state['a'] else 0.0
        self.force_z = self.current_thrust if self.key_state['i'] else -self.current_thrust if self.key_state['k'] else 0.0
        self.torque_z = MAX_TORQUE if self.key_state['j'] else -MAX_TORQUE if self.key_state['l'] else 0.0

    def timer_callback(self):
        self.wrench = np.array([self.force_x, self.force_y, self.force_z, 0.0, 0.0, self.torque_z])
        msg = Wrench()
        msg.force = Vector3(x=self.force_x, y=self.force_y, z=self.force_z)
        msg.torque = Vector3(x=0.0, y=0.0, z=self.torque_z)
        self.publisher_.publish(msg)

        self.get_logger().info(f"Publishing: {self.wrench}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()

    # Register the signal handler for clean shutdown
    signal.signal(signal.SIGINT, node.signal_handler)
    signal.signal(signal.SIGTERM, node.signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.signal_handler(signal.SIGINT, None)

if __name__ == "__main__":
    main()
