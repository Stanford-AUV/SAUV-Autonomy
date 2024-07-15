import numpy as np
import json

"""
Robot Orientation
"""

def quadratic_model(x, a, b):
    return a * np.square(x - b)


def inverse_quadratic_model(x, a, b):
    second_part = np.sqrt(np.abs(x)) / np.sqrt(a)
    return np.where(x >= 0, b + second_part, b - second_part)


COEFFS = {
    10: [1.8343560975506323e-05, 1481.466452853926],
    12: [2.3194713277650457e-05, 1481.0593557569864],
    14: [2.8235512746477604e-05, 1480.8191181032553],
    16: [3.28040783880607e-05, 1480.6115720889093],
    18: [3.638563822982247e-05, 1481.421310875163],
    20: [3.8972515653759295e-05, 1480.4018523627565],
}

TAM = np.empty(shape=(6, 8))

# thruster positions in body frame
# Real robot axes, the axes in Gazebo are different

"""
Frame measurements:
x length: 0.56m. Motors on frame
y length: 0.30m, Add/subtract 0.076m to get motor positions
z length: 0.43m. Motors 0.09m and 0.254m off the ground

Motor positions are meant to be relative to CG
For ease of measurement we assume CG is at the center of the frame
"""

r_is = np.array(
    [
        [-0.272, +0.22, -0.125],  # ... pin 2 bottom, lower right
        [+0.272, +0.22, -0.125],  # ... pin 3 bottom, upper right
        [+0.272, -0.22, -0.125],  # ... pin 4 bottom, upper left
        [-0.272, -0.22, -0.125],  # ... pin 5 bottom, lower left
        [-0.272, -0.22, 0.04],  # ... pin 6 top, lower left
        [-0.272, +0.22, 0.04],  # ... pin 7 top, lower right
        [+0.272, +0.22, 0.04],  # ... pin 8 top, upper right
        [+0.272, -0.22, 0.04],  # ... pin 9, top, upper left
    ]
)

# thruster orientations
# note: these point in the direction of positive thrust; z axis can be changed accordingly

u_is = np.array(
    [
        [0, 0, -1],
        [0, 0, -1],
        [0, 0, -1],
        [0, 0, 1], # pin 5, weak. perhaps faulty. check cables?
        [-1, 1, 0],
        [-1, -1, 0],
        [-1, 1,0],
        [-1, -1, 0],
    ]
)

u_is = u_is / np.linalg.norm(u_is, axis=1)[:, None]

TAM[:3, :] = u_is.T
TAM[3:, :] = np.cross(r_is, u_is).T

TAM_inv = np.linalg.pinv(TAM)


def total_force_to_individual_thrusts(desired_wrench):
    """Converts a desired force to motor thrusts.
    Force is a 6x1 vector with the desired force in the x, y, z, roll, pitch, and yaw directions.
    """
    return TAM_inv @ desired_wrench


def thrust_to_pwm(thrust, voltage=14.8):
    """Converts a desired thrust to motor PWM values."""

    if voltage > 20:
        raise ValueError("Voltage exceeds the maximum allowed limit of 20V.")
    elif voltage < 10:
        raise ValueError("Voltage is below the minimum allowed voltage of 10V.")

    # Requested thrust must be 20% below saturation limit, according to https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/
    if thrust > 0.8 * (0.374 * voltage - 0.78):  # mx + b, calculated by hand
        raise ValueError("Forward thrust exceeds the maximum limit")
    if -thrust > 0.8 * (0.266 * voltage - 0.272):  # mx + b, calculated by hand
        raise ValueError("Reverse thrust exceeds the maximum limit")

    if voltage >= 18:
        low = 18
        high = 20
    elif voltage >= 16:
        low = 16
        high = 18
    elif voltage >= 14:
        low = 14
        high = 16
    elif voltage >= 12:
        low = 12
        high = 14
    else:
        low = 10
        high = 12

    weight = (thrust - low) / 2
    low_pwm = inverse_quadratic_model(thrust, *COEFFS[low])
    high_pwm = inverse_quadratic_model(thrust, *COEFFS[high])
    pwm = (1 - weight) * low_pwm + weight * high_pwm

    return np.array(np.round(pwm), dtype=int)


def thrusts_to_pwm(thrusts):
    """Converts a desired array of thrusts to motor PWM values."""
    return np.array([thrust_to_pwm(thrust) for thrust in thrusts])


if __name__ == "__main__":
    print(thrust_to_pwm(0, voltage=18))
    print(thrust_to_pwm(0, voltage=15))
    print(thrust_to_pwm(0, voltage=12))

    print(thrust_to_pwm(2, voltage=18))
    print(thrust_to_pwm(2, voltage=15))
    print(thrust_to_pwm(2, voltage=12))

    print(thrust_to_pwm(-2, voltage=18))
    print(thrust_to_pwm(-2, voltage=15))
    print(thrust_to_pwm(-2, voltage=12))

    # TODO test thrust_to_pwm using margins of errors
    # print(total_force_to_individual_pwm(np.array([1, 0, 0, 0, 0, 0])))
