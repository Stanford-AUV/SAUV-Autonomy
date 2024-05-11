from simple_pid import PID
from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt


class Controller:
    def __init__(self, p_values, i_values, d_values):
        """
        Initialize the Controller with PID parameters for position and orientation.

        :param p_values: List of P values for the PID controllers [px, py, pz, p_roll, p_pitch, p_yaw].
        :param i_values: List of I values for the PID controllers [ix, iy, iz, i_roll, i_pitch, i_yaw].
        :param d_values: List of D values for the PID controllers [dx, dy, dz, d_roll, d_pitch, d_yaw].
        """
        self.position_pids = [
            PID(p, i, d, sample_time=0)
            for p, i, d in zip(p_values[:3], i_values[:3], d_values[:3])
        ]
        self.orientation_pids = [
            PID(p, i, d, sample_time=0)
            for p, i, d in zip(p_values[3:], i_values[3:], d_values[3:])
        ]
        self.pids = self.position_pids + self.orientation_pids

    def update(self, current_position, current_orientation):
        """
        Update the PID controllers with the current state.

        :param current_position: Current position [x, y, z].
        :param current_orientation: Current orientation as a unit quaternion [qx, qy, qz, qw].
        :return: List of control outputs [force_x, force_y, force_z, torque_roll, torque_pitch, torque_yaw].
        """

        # Compute position control outputs
        force_outputs = np.array(
            [pid(current) for pid, current in zip(self.position_pids, current_position)]
        )

        torque_outputs = np.array(
            [
                pid(current)
                for pid, current in zip(self.orientation_pids, current_orientation)
            ]
        )

        return np.concatenate((force_outputs, torque_outputs))

    def set_position_setpoint(self, position_setpoint):
        """
        Update the position setpoint of the PID controllers.

        :param position_setpoint: New target position [x, y, z].
        """
        for pid, sp in zip(self.position_pids, position_setpoint):
            pid.setpoint = sp

    def set_orientation_setpoint(self, orientation_setpoint):
        """
        Update the orientation setpoint.

        :param orientation_setpoint: New target orientation as a unit quaternion [qx, qy, qz, qw].
        """
        for pid, sp in zip(self.orientation_pids, orientation_setpoint):
            pid.setpoint = sp

    def reset(self):
        """
        Reset all PID controllers to their initial states.
        """
        for pid in self.pids:
            pid.reset()


# Example usage with simulation
def simulate_controller(
    controller, initial_position, initial_orientation, timesteps, dt, mass, inertia
):
    """
    Simulate the controller over a given number of timesteps.

    :param controller: The Controller instance.
    :param initial_position: Initial position [x, y, z].
    :param initial_orientation: Initial orientation as Euler angles [roll, pitch, yaw].
    :param timesteps: Number of simulation steps.
    :param dt: Time step duration.
    :param mass: Mass of the object being controlled.
    :param inertia: Inertia of the object [I_roll, I_pitch, I_yaw].
    :return: Tuple (time, positions, orientations, control_outputs) where positions, orientations, and control_outputs are lists over time.
    """
    time = np.linspace(0, timesteps * dt, timesteps)
    positions = np.zeros((timesteps, 3))
    orientations = np.zeros((timesteps, 3))
    control_outputs = np.zeros((timesteps, 6))

    positions[0] = initial_position
    orientations[0] = initial_orientation
    velocity = np.zeros(6)

    for t in range(1, timesteps):
        if t == 0:
            controller.set_position_setpoint(np.array([0, 0, 0]))
            controller.set_orientation_setpoint(np.array([0.7071, 0, 0.7071, 0]))
        if t == timesteps // 2:
            controller.set_position_setpoint(np.array([10, 10, 10]))
            controller.set_orientation_setpoint(np.array([0, 0, 0, 1]))

        control_output = controller.update(positions[t - 1], orientations[t - 1])
        control_outputs[t] = control_output

        # Update linear velocity and position
        acceleration = np.array(control_output[:3]) / mass
        velocity[:3] += acceleration * dt
        positions[t, :3] = positions[t - 1, :3] + velocity[:3] * dt

        # Update angular velocity and orientation
        angular_acceleration = np.array(control_output[3:]) / np.array(inertia)
        velocity[3:] += angular_acceleration * dt
        delta_orientation = R.from_rotvec(velocity[3:] * dt)
        orientations[t] = (
            R.from_euler("xyz", orientations[t - 1]) * delta_orientation
        ).as_euler("xyz")

    return time, positions, orientations, control_outputs


# Define PID parameters and setpoints
p_values = np.array([0.5, 0.5, 0.5, 0.2, 0.2, 0.2])
i_values = np.array([0, 0, 0, 0, 0, 0])
d_values = np.array([0.005, 0.005, 0.005, 0.005, 0.005, 0.005])
position_setpoint = np.array([0, 0, 0])
orientation_setpoint = np.array([0, 0, 0])

# Create controller instance
controller = Controller(p_values, i_values, d_values)

# Initial state
initial_position = np.array([5, 5, 5])
initial_orientation = np.array([0, 0, 0])

# Simulation parameters
timesteps = 3000
dt = 0.01
mass = 1.0
inertia = np.array([0.005, 0.005, 0.005])

# Run simulation
time, positions, orientations, control_outputs = simulate_controller(
    controller, initial_position, initial_orientation, timesteps, dt, mass, inertia
)

# Plotting
fig, axs = plt.subplots(6, 1, figsize=(10, 15), sharex=True)
components = [
    "X",
    "Y",
    "Z",
    "Roll",
    "Pitch",
    "Yaw",
]

# Plot positions for x, y, z
for i in range(3):
    axs[i].plot(time, positions[:, i], label=f"Position {components[i]}")
    axs[i].plot(
        time,
        control_outputs[:, i],
        label=f"Control Output {components[i]}",
        linestyle="--",
    )
    axs[i].set_ylabel(f"{components[i]}")
    axs[i].legend()
    axs[i].grid(True)

# Plot control outputs for torques roll, pitch, yaw
for i in range(3, 6):
    axs[i].plot(time, orientations[:, i - 3], label=f"Rotation {components[i]}")
    axs[i].plot(
        time,
        control_outputs[:, i],
        label=f"Control Output {components[i]}",
        linestyle="--",
    )
    axs[i].set_ylabel(f"{components[i]}")
    axs[i].legend()
    axs[i].grid(True)

axs[-1].set_xlabel("Time (s)")

plt.tight_layout()
plt.show()
