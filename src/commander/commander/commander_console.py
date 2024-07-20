import sys
import threading
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

from srvs.srv import MotorEnable
from rclpy.node import Node

msg = """
Missing init message!

CTRL-C to quit
"""

class CommanderNode(Node):
    def __init__(self):
        super().__init__('commander_console')

        self.cli = self.create_client(MotorEnable, 'motor_enable')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MotorEnable (Arduino node) service not available, waiting again...')
        self.motor_req = MotorEnable.Request()
        self.get_logger().info('MotorEnable (Arduino node) service available!')

    def send_motor_enable_request(self, enable):
        self.motor_req.enable = enable
        self.future = self.cli.call_async(self.motor_req)
        return self.future

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()

    rclpy.init()

    cmd_node = CommanderNode()

    try:
        while rclpy.ok():
            key = getKey(settings)
            if key == "x" or key == "X":
                future = cmd_node.send_motor_enable_request(enable=False)
                rclpy.spin_until_future_complete(cmd_node, future)
                response = future.result()
                cmd_node.get_logger().info(f'Motor disabled: {response.success}')
            elif key == "g":
                future = cmd_node.send_motor_enable_request(enable=True)
                rclpy.spin_until_future_complete(cmd_node, future)
                response = future.result()
                cmd_node.get_logger().info(f'Motor enabled: {response.success}')
            else:
                if key == '\x03':
                    break

            # Spin once to process other callbacks
            rclpy.spin_once(cmd_node, timeout_sec=0.1)

    except Exception as e:
        print(e)

    finally:
        cmd_node.destroy_node()
        rclpy.shutdown()
        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()
