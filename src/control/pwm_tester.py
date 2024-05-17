import numpy as np
import serial
import time
from control.force_to_pwm import total_force_to_individual_thrusts, thrusts_to_pwm

def main():
    portName = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
    thruster_ids = [f"thruster{i}" for i in range(1, 9)]
    testing_wrench = np.array([1, 0, 0, 0, 0, 0])
    thrusts = total_force_to_individual_thrusts(testing_wrench)
    pwms = thrusts_to_pwm(thrusts)
    print(f'pwms: {pwms}')

    n = 3
    for j in range(n):
        if j == n-1:
            time.sleep(5)
        for i, thruster in enumerate(thruster_ids):
            if j == n - 1:
                command = f"{i+2} 1500\n"
            else:
                command = f"{i+2} {forward[i]}\n"
            print(f"command: {command}")
            portName.write(command.encode())
            ##time.sleep(.1)

if __name__ == '__main__':
    main()
