"""Phase 2 behavior and analytical checks."""
import unittest
from water_tank.feedback_control import (
    ControllerParameters,
    inlet_command,
    settling_time,
    simulate,
    steady_state_height,
)
from water_tank.tank_simulation import TankParameters


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.p = TankParameters()
        self.c = ControllerParameters()

    def test_initial_control(self):
        self.assertAlmostEqual(inlet_command(1.0, self.c), 0.095)

    def test_actuator_limits(self):
        self.assertEqual(inlet_command(-100, self.c), self.c.max_flow)
        self.assertEqual(inlet_command(100, self.c), self.c.min_flow)

    def test_bias_maintains_target(self):
        self.assertAlmostEqual(steady_state_height(self.p, self.c), 3.0)
        at_target = TankParameters(initial_height=3.0)
        sim = simulate(at_target, self.c, duration=100, dt=0.1)
        self.assertAlmostEqual(sim['height'][-1], 3.0, places=9)

    def test_no_bias_has_offset(self):
        c = ControllerParameters(bias=0.0)
        expected = (c.kp*c.target - self.p.leak_flow)/(self.p.outlet_coefficient+c.kp)
        self.assertAlmostEqual(steady_state_height(self.p, c), expected)
        self.assertAlmostEqual(expected, 2.125)

    def test_disturbance_has_offset(self):
        self.assertAlmostEqual(steady_state_height(self.p, self.c, extra_leak=0.01), 2.75)
        sim = simulate(self.p, self.c, disturbance_at=600, extra_leak=0.01)
        self.assertAlmostEqual(sim['height'][-1], 2.75, delta=0.002)

    def test_feedback_reaches_target(self):
        sim = simulate(self.p, self.c)
        self.assertAlmostEqual(sim['height'][-1], 3.0, delta=0.001)
        self.assertIsNotNone(settling_time(sim['time'], sim['height'], 3.0))

    def test_high_gain_saturates(self):
        c = ControllerParameters(kp=1.0)
        self.assertAlmostEqual(inlet_command(1.0, c), 0.10)


if __name__ == '__main__':
    unittest.main()
