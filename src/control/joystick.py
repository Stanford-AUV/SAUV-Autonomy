from gamepad import Gamepad
import numpy as np
import sys
import time

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


if __name__ == "__main__":
    pad = None

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
