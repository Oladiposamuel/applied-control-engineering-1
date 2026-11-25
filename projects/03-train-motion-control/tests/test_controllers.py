import unittest

from train_motion.anti_windup import conditional_integral_rate
from train_motion.controllers import ControllerParameters, command
from train_motion.model import PlantParameters, TrainState


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.plant = PlantParameters()

    def test_speed_integral_units_and_zero_error_memory(self):
        result = command(TrainState(0, 25), 30, ControllerParameters(mode="speed_pi"), self.plant)
        self.assertEqual(result.integral_units, "m")
        self.assertEqual(result.integral_rate, 0)
        self.assertEqual(result.integral, 3_000)

    def test_position_integral_units(self):
        result = command(TrainState(400, 20), 200, ControllerParameters(mode="position_pid"), self.plant)
        self.assertEqual(result.integral_units, "m*s")

    def test_conditional_anti_windup_both_limits_and_unwinding(self):
        self.assertEqual(conditional_integral_rate(10, 50_000, 5, -40_000, 40_000), (0, True))
        self.assertEqual(conditional_integral_rate(-10, 50_000, 5, -40_000, 40_000), (-10, False))
        self.assertEqual(conditional_integral_rate(-10, -50_000, 5, -40_000, 40_000), (0, True))
        self.assertEqual(conditional_integral_rate(10, -50_000, 5, -40_000, 40_000), (10, False))

    def test_position_aw_blocks_initial_saturation(self):
        result = command(TrainState(0, 0), 0, ControllerParameters(mode="position_pid_aw"), self.plant)
        self.assertTrue(result.anti_windup_active)
        self.assertEqual(result.integral_rate, 0)

    def test_mode_validation(self):
        with self.assertRaises(ValueError):
            ControllerParameters(mode="wrong")


if __name__ == "__main__":
    unittest.main()
