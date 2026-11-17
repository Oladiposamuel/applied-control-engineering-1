"""Tests for the open-loop tank model."""
import unittest
import numpy as np
from water_tank.tank_simulation import (
    TankParameters,
    analytical_height,
    equilibrium_height,
    forward_euler,
    height_rate,
)


class TankTests(unittest.TestCase):
    def setUp(self):
        self.p = TankParameters()

    def test_equilibrium(self):
        self.assertAlmostEqual(equilibrium_height(self.p), 3.5)
        self.assertAlmostEqual(height_rate(3.5, self.p), 0.0)

    def test_initial_rate(self):
        self.assertAlmostEqual(height_rate(1.0, self.p), 0.0125)

    def test_first_two_euler_steps(self):
        _, h = forward_euler(self.p, 0.2, 0.1)
        self.assertAlmostEqual(h[1], 1.00125)
        self.assertAlmostEqual(h[2], 1.002499375)

    def test_euler_agrees_with_exact(self):
        t, h = forward_euler(self.p, 1000, 0.1)
        self.assertLess(np.max(np.abs(h - analytical_height(t, self.p))), 0.00025)


if __name__ == '__main__':
    unittest.main()
