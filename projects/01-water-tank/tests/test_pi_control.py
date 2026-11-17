"""Phase 3 PI controller arithmetic, dynamics, and anti-windup tests."""
import unittest

import numpy as np

from water_tank.feedback_control import ControllerParameters, simulate
from water_tank.pi_control import (
    PIControllerParameters,
    integral_rate,
    pi_command,
    simulate_pi,
)
from water_tank.tank_simulation import TankParameters


class PIControlTests(unittest.TestCase):
    def setUp(self):
        self.plant = TankParameters()
        self.control = PIControllerParameters()

    def test_pi_command_arithmetic(self):
        raw, actual, saturated = pi_command(2.5, 4.0, self.control)
        self.assertAlmostEqual(raw, 0.049)
        self.assertAlmostEqual(actual, 0.049)
        self.assertFalse(saturated)

    def test_first_step_uses_simultaneous_old_states(self):
        plant = TankParameters(initial_height=2.5)
        run = simulate_pi(plant, self.control, duration=0.1, dt=0.1,
                          initial_integral=4.0)
        self.assertAlmostEqual(run["height"][1], 2.50095)
        self.assertAlmostEqual(run["integral_state"][1], 4.05)

    def test_nominal_equilibrium(self):
        plant = TankParameters(initial_height=3.0)
        run = simulate_pi(plant, self.control, duration=10.0, dt=0.1)
        np.testing.assert_allclose(run["height"], 3.0, atol=1e-12)
        np.testing.assert_allclose(run["integral_state"], 0.0, atol=1e-12)
        np.testing.assert_allclose(run["flow"], 0.035, atol=1e-12)

    def test_disturbed_equilibrium(self):
        plant = TankParameters(initial_height=3.0)
        run = simulate_pi(plant, self.control, duration=10.0, dt=0.1,
                          disturbance_at=0.0, extra_leak=0.007,
                          initial_integral=7.0)
        np.testing.assert_allclose(run["height"], 3.0, atol=1e-12)
        np.testing.assert_allclose(run["integral_state"], 7.0, atol=1e-12)
        np.testing.assert_allclose(run["flow"], 0.042, atol=1e-12)

    def test_p_controller_retains_disturbance_offset(self):
        plant = TankParameters(initial_height=3.0)
        p_control = ControllerParameters(kp=0.02, target=3.0, bias=0.035)
        run = simulate(plant, p_control, duration=2400, dt=0.1,
                       disturbance_at=600, extra_leak=0.007)
        expected = 0.083 / 0.03
        self.assertLess(run["height"][-1], 3.0)
        self.assertAlmostEqual(run["height"][-1], expected, delta=0.001)

    def test_pi_rejects_persistent_disturbance(self):
        plant = TankParameters(initial_height=3.0)
        run = simulate_pi(plant, self.control, duration=2400, dt=0.1,
                          disturbance_at=600, extra_leak=0.007)
        self.assertAlmostEqual(run["height"][-1], 3.0, delta=0.01)
        self.assertAlmostEqual(run["integral_state"][-1], 7.0, delta=0.1)
        self.assertAlmostEqual(run["flow"][-1], 0.042, delta=0.0001)

    def test_flow_is_clipped_at_both_limits(self):
        high = pi_command(-100, 100, self.control)
        low = pi_command(100, -100, self.control)
        self.assertEqual(high[1], self.control.max_flow)
        self.assertEqual(low[1], self.control.min_flow)
        self.assertTrue(high[2] and low[2])

    def test_conditional_anti_windup_blocks_outward_integration(self):
        self.assertEqual(integral_rate(2.0, 0.2, self.control), 0.0)
        self.assertEqual(integral_rate(-2.0, -0.1, self.control), 0.0)
        no_aw = PIControllerParameters(anti_windup="none")
        self.assertEqual(integral_rate(2.0, 0.2, no_aw), 2.0)
        self.assertEqual(integral_rate(-2.0, -0.1, no_aw), -2.0)

    def test_conditional_anti_windup_allows_recovery(self):
        self.assertEqual(integral_rate(-1.0, 0.2, self.control), -1.0)
        self.assertEqual(integral_rate(1.0, -0.1, self.control), 1.0)
        self.assertEqual(integral_rate(0.5, 0.05, self.control), 0.5)

    def test_ki_zero_matches_p_flow(self):
        pi = PIControllerParameters(kp=0.02, ki=0.0)
        p = ControllerParameters(kp=0.02)
        pi_run = simulate_pi(self.plant, pi, duration=20, dt=0.1,
                             initial_integral=999.0)
        p_run = simulate(self.plant, p, duration=20, dt=0.1)
        np.testing.assert_allclose(pi_run["flow"], p_run["flow"])
        np.testing.assert_allclose(pi_run["height"], p_run["height"])

    def test_array_alignment(self):
        run = simulate_pi(self.plant, self.control, duration=1.0, dt=0.1)
        self.assertEqual(run["steps"], 10)
        for key in ("time", "height", "integral_state", "error"):
            self.assertEqual(len(run[key]), 11)
        for key in ("raw_flow", "flow", "leak", "saturated"):
            self.assertEqual(len(run[key]), 10)

    def test_input_validation(self):
        bad_controllers = (
            dict(kp=-1), dict(ki=-1), dict(max_flow=0),
            dict(kp=float("nan")), dict(anti_windup="unknown"),
        )
        for kwargs in bad_controllers:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                PIControllerParameters(**kwargs)
        bad_runs = (
            dict(duration=0), dict(dt=0), dict(duration=1, dt=0.3),
            dict(extra_leak=-1), dict(initial_integral=float("inf")),
            dict(disturbance_at=2000),
        )
        for kwargs in bad_runs:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                simulate_pi(self.plant, self.control, **kwargs)


if __name__ == "__main__":
    unittest.main()
