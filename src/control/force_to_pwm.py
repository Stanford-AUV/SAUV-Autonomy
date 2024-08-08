import numpy as np
import json

"""
Robot Orientation
"""

COEFFS = {
        10: [1.8343560975506323e-05, 1481.466452853926],
        12: [2.3194713277650457e-05, 1481.0593557569864],
        14: [2.8235512746477604e-05, 1480.8191181032553],
        16: [3.28040783880607e-05, 1480.6115720889093],
        18: [3.638563822982247e-05, 1481.421310875163],
        20: [3.8972515653759295e-05, 1480.4018523627565],
    }

def quadratic_model(x, a, b):
    return a * np.square(x - b)


def inverse_quadratic_model(x, a, b):
    second_part = np.sqrt(np.abs(x)) / np.sqrt(a)
    return np.where(x >= 0, b + second_part, b - second_part)


def total_force_to_individual_thrusts(desired_wrench):
    """Converts a desired force to motor thrusts.
    Force is a 6x1 vector with the desired force in the x, y, z, roll, pitch, and yaw directions.
    """

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
            [-0.272, -0.22, -0.125],  # ... pin 2 bottom, lower right
            [+0.272, -0.22, -0.125],  # ... pin 3 bottom, upper right
            [+0.272, +0.22, -0.125],  # ... pin 4 bottom, upper left
            [-0.272, +0.22, -0.125],  # ... pin 5 bottom, lower left
            [-0.272, +0.22, +0.04],  # ... pin 6 top, lower left
            [-0.272, -0.22, +0.04],  # ... pin 7 top, lower right
            [+0.272, -0.22, +0.04],  # ... pin 8 top, upper right
            [+0.272, +0.22, +0.04],  # ... pin 9, top, upper left
        ]
    )

    # thruster orientations
    # note: these point in the direction of positive thrust; z axis can be changed accordingly

    u_is = np.array(
        [
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [-np.cos(np.pi / 4), -np.sin(np.pi / 4), 0],
            [-np.cos(np.pi / 4), +np.sin(np.pi / 4), 0],
            [-np.cos(np.pi / 4), -np.sin(np.pi / 4), 0],
            [-np.cos(np.pi / 4), +np.sin(np.pi / 4), 0],
        ]
    )

    u_is = u_is / np.linalg.norm(u_is, axis=1)[:, None]

    TAM[:3, :] = u_is.T
    TAM[3:, :] = np.cross(r_is, u_is).T

    TAM_inv = np.linalg.pinv(TAM)
    return TAM_inv @ desired_wrench


def thrust_to_pwm(thrust, voltage=15.0):
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

    to_add = 0  # should be centered around 1500, inverse quad model is centering around 1482 for 14-16V
    if voltage >= 18:
        low = 18
        high = 20
    elif voltage >= 16:
        low = 16
        high = 18
        to_add = 26
    elif voltage >= 14:
        low = 14
        high = 16
        to_add = 18
    elif voltage >= 12:
        low = 12
        high = 14
    else:
        low = 10
        high = 12

    weight = (thrust - low) / 2
    low_pwm = inverse_quadratic_model(thrust, *COEFFS[low])
    high_pwm = inverse_quadratic_model(thrust, *COEFFS[high])
    pwm = (1 - weight) * low_pwm + weight * high_pwm + to_add

    return np.array(np.round(pwm), dtype=int)


def all_thrusts_to_pwm(thrusts):
    """Converts a desired array of thrusts to motor PWM values."""
    return np.array([thrust_to_pwm(thrust) for thrust in thrusts])


def test(thrust_distribution=False, pwm_distribution=False):
    thrust0 = [0, 0, 0, 0, 0, 0]
    thrust1 = [1, 0, 0, 0, 0, 0]
    thrust2 = [0, 1, 0, 0, 0, 0]
    thrust3 = [0, 0, 1, 0, 0, 0]
    thrust4 = [0, 0, 0, 0, 0, 1]

    # Test correct allocation of thrusts
    if thrust_distribution or pwm_distribution:
        ind0 = total_force_to_individual_thrusts(thrust0)
        ind1 = total_force_to_individual_thrusts(thrust1)
        ind2 = total_force_to_individual_thrusts(thrust2)
        ind3 = total_force_to_individual_thrusts(thrust3)
        ind4 = total_force_to_individual_thrusts(thrust4)
        if thrust_distribution:
            print(
                f"distributed thrusts:\nnone:\n{ind0}\n\nx:\n{ind1}\n\ny:\n{ind2}\n\nz:\n{ind3}\n\nyaw:\n{ind4}\n"
            )

    # Check correct pwm calculation
    if pwm_distribution:
        pwm0 = all_thrusts_to_pwm(ind0)
        pwm1 = all_thrusts_to_pwm(ind1)
        pwm2 = all_thrusts_to_pwm(ind2)
        pwm3 = all_thrusts_to_pwm(ind3)
        pwm4 = all_thrusts_to_pwm(ind4)
        print(
            f"pwm arrays:\nnone:\n{pwm0}\n\nx:\n{pwm1}\n\ny\n{pwm2}\n\nz\n{pwm3}\n\nyaw\n{pwm4}\n"
        )


if __name__ == "__main__":
    test()
