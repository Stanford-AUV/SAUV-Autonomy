import numpy as np

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


def thrust_to_pwm(force, voltage=14.8):
    """Converts a desired force to motor PWM values.
    Force is a
    """
    pwm = np.sign(force) * np.sqrt(30000 * np.abs(force)) + 1500
    if voltage != 14.8:
        raise NotImplementedError("Only 14.8V is supported at the moment.")
    return np.array(np.round(pwm), dtype=int)


def total_force_to_individual_pwm(desired_wrench):
    """Converts a desired force to motor PWM values.
    Force is a 6x1 vector with the desired force in the x, y, z, roll, pitch, and yaw directions.
    """
    return thrust_to_pwm(total_force_to_individual_thrusts(desired_wrench))


if __name__ == "__main__":
    print(total_force_to_individual_pwm(np.array([1, 0, 0, 0, 0, 0])))
