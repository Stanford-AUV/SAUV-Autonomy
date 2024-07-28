import numpy as np
import serial
import time
from control.force_to_pwm import total_force_to_individual_thrusts, thrusts_to_pwm


def main():
    portName = serial.Serial("/dev/ttyUSB_teensy", baudrate=9600, timeout=1)
    thruster_ids = [f"thruster{i}" for i in range(1, 9)]
    testing_wrench = np.array([0, -0.0, 0, 0, 0, 0])
    thrusts = total_force_to_individual_thrusts(testing_wrench)
    pwms = thrusts_to_pwm(thrusts)
    # pwms = [1500, 1500, 1500, 1500, 1450, 1550, 1450, 1550] # specify pwms here
    print(f"pwms: {pwms}")

    n = 20
    # for i in range(8):
    #   time.sleep(10)
    #   print(f"{80 - i * 10} seconds left!")
    for j in range(n):
        time.sleep(5)
        for i, thruster in enumerate(thruster_ids):
            if j == n - 1:
                command = f"{i+2} 1500\n"
            else:
                command = f"{i+2} {pwms[i]}\n"
            portName.write(command.encode())
            ##time.sleep(.1)


if __name__ == "__main__":
    main()
