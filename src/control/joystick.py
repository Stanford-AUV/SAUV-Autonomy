import rclpy
from rclpy.node import Node
import numpy as np
import sys
import time

from control.gamepad import Gamepad

from msgs.msg import Wrench
from geometry_msgs.msg import Vector3

# Thrust
# [fx, fy, fz, tx, ty, tz]
# tz --> left joystick x axis
# fx -> right joystick y axis
# fy -> right joystick x axis
# fz -> left joystick y axis

MAX_THRUST = 0.8
MAX_TORQUE = 0.1
DEAD_LOCK = 10


def dead_lock(value):
    if abs(value - 127.5) <= DEAD_LOCK:
        return 127.5
    return value


def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def loop():
    pad = Gamepad()
    record = "--record" in sys.argv

    file = None
    if record:
        file = open("wrench.txt", "w")

    try:
        while True:
            pad.read_gamepad()
            # if pad.changed:
            Lx = dead_lock(pad.get_analogL_x())
            Ly = dead_lock(pad.get_analogL_y())
            Rx = dead_lock(pad.get_analogR_x())
            Ry = dead_lock(pad.get_analogR_y())

            fx = map_value(Ry, 0, 255, -MAX_THRUST, MAX_THRUST)
            fy = -map_value(Rx, 0, 255, -MAX_THRUST, MAX_THRUST)
            fz = map_value(Ly, 0, 255, -MAX_THRUST, MAX_THRUST)
            tz = map_value(Lx, 0, 255, -MAX_TORQUE, MAX_TORQUE)
            wrench = np.array([fx, fy, fz, 0, 0, tz])

            print(wrench)
            if record:
                timestamp = time.time_ns()

            if record:
                file.write(f"{timestamp}: {wrench}\n")
                file.flush()
    finally:
        if record:
            file.close()


class Joystick(Node):
    def __init__(self):
        super().__init__("joystick")
        self.publisher_ = self.create_publisher(Wrench, "wrench", 10)
        self.pad = Gamepad()
        self.record = "--record" in sys.argv
        self.file = open("wrench.txt", "w") if self.record else None

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.pad.read_gamepad()

        Lx = dead_lock(self.pad.get_analogL_x())
        Ly = dead_lock(self.pad.get_analogL_y())
        Rx = dead_lock(self.pad.get_analogR_x())
        Ry = dead_lock(self.pad.get_analogR_y())

        fx = map_value(Ry, 0, 255, -MAX_THRUST, MAX_THRUST)
        fy = -map_value(Rx, 0, 255, -MAX_THRUST, MAX_THRUST)
        fz = map_value(Ly, 0, 255, -MAX_THRUST, MAX_THRUST)
        tz = map_value(Lx, 0, 255, -MAX_TORQUE, MAX_TORQUE)

        wrench = np.array([fx, fy, fz, 0.0, 0.0, tz])
        msg = Wrench()
        msg.force = Vector3(x=fx, y=fy, z=fz)
        msg.torque = Vector3(x=0.0, y=0.0, z=tz)
        self.publisher_.publish(msg)

        self.get_logger().info(f"Publishing: {wrench}")

        if self.record:
            timestamp = time.time_ns()
            self.file.write(f"{timestamp}: {wrench}\n")
            self.file.flush()

    def destroy_node(self):
        if self.record:
            self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Joystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
