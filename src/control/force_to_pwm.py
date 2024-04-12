import numpy as np
import json
from scripts.find_pwm_function import inverse_quadratic_model

filename = "coeffs.json"
with open(filename, "r") as f:
    COEFFS = json.load(f)
COEFFS = {int(k): v for k, v in COEFFS.items()}  # convert keys from strings to ints

TAM = np.empty(shape=(6, 4))

# thruster positions in body frame
r_is = np.array([[1, 1, 0], [-1, 1, 0], [-1, -1, 0], [1, -1, 0]])

# thruster orientations
# note: these point in the direction of positive thrust
u_is = np.array(
    [
        [-np.cos(np.pi / 4), np.sin(np.pi / 4), 0],
        [np.cos(np.pi / 4), np.sin(np.pi / 4), 0],
        [np.cos(np.pi / 4), -np.sin(np.pi / 4), 0],
        [-np.cos(np.pi / 4), -np.sin(np.pi / 4), 0],
    ]
)

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


def total_force_to_individual_pwm(desired_wrench):
    """Converts a desired force to motor PWM values.
    Force is a 6x1 vector with the desired force in the x, y, z, roll, pitch, and yaw directions.
    """
    return thrust_to_pwm(total_force_to_individual_thrusts(desired_wrench))


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
