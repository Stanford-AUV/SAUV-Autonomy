import unittest
from simple_pid import PID
import numpy as np

from control.controller import Controller


class TestController(unittest.TestCase):
    def setUp(self):
        p_values = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])
        i_values = np.array([0.1, 0.1, 0.1, 0.05, 0.05, 0.05])
        d_values = np.array([0.01, 0.01, 0.01, 0.005, 0.005, 0.005])
        self.controller = Controller(p_values, i_values, d_values)

    def test_initialization(self):
        self.assertEqual(len(self.controller.pids), 6)
        for pid in self.controller.pids:
            self.assertIsInstance(pid, PID)
            self.assertIn(pid.setpoint, np.array([0, 0, 0, 0, 0, 0]))

    def test_update(self):
        wrench = np.array([0.5, -0.3, 0.1, 0.02, -0.01, 0.05])
        expected_outputs = -np.array(
            [
                0.5 * 1.0 + 0.5 * 0.1 + 0.5 * 0.01,  # PID output for x-axis
                -0.3 * 1.0 + -0.3 * 0.1 + -0.3 * 0.01,  # PID output for y-axis
                0.1 * 1.0 + 0.1 * 0.1 + 0.1 * 0.01,  # PID output for z-axis
                0.02 * 0.5 + 0.02 * 0.05 + 0.02 * 0.005,  # PID output for roll
                -0.01 * 0.5 + -0.01 * 0.05 + -0.01 * 0.005,  # PID output for pitch
                0.05 * 0.5 + 0.05 * 0.05 + 0.05 * 0.005,  # PID output for yaw
            ]
        )
        outputs = self.controller.update(wrench[:3], wrench[3:])
        np.testing.assert_allclose(outputs, expected_outputs, rtol=1e-1)

    def test_set_setpoints(self):
        new_setpoints = np.array([1, 1, 1, 0.5, 0.5, 0.5])
        self.controller.set_position_setpoint(new_setpoints[:3])
        self.controller.set_orientation_setpoint(new_setpoints[3:])
        for pid, setpoint in zip(self.controller.pids, new_setpoints):
            self.assertEqual(pid.setpoint, setpoint)

    def test_reset(self):
        self.controller.reset()
        for pid in self.controller.pids:
            p, i, d = pid.components
            self.assertAlmostEqual(p, 0.0, delta=1e-6)
            self.assertAlmostEqual(i, 0.0, delta=1e-6)
            self.assertAlmostEqual(d, 0.0, delta=1e-6)


# Run the tests
if __name__ == "__main__":
    unittest.main()
