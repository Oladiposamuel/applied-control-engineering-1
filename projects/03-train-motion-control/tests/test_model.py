import unittest

from train_motion.model import (PlantParameters, TrainState, clip_force,
                                exact_open_loop, force_balance,
                                forward_euler_step)


class ModelTests(unittest.TestCase):
    def setUp(self):
        self.plant = PlantParameters()

    def test_signed_saturation(self):
        self.assertEqual(clip_force(50_000, self.plant), (40_000, True))
        self.assertEqual(clip_force(-50_000, self.plant), (-40_000, True))
        self.assertEqual(clip_force(-6_000, self.plant), (-6_000, False))

    def test_standstill_rule(self):
        for request in (-40_000, 0, 1_999, 2_000):
            result = force_balance(0, request, 0, self.plant)
            self.assertEqual(result.acceleration, 0)
            self.assertTrue(result.standstill_constraint)
        moving = force_balance(0, 3_000, 0, self.plant)
        self.assertAlmostEqual(moving.acceleration, .05)

    def test_disturbance_raises_standstill_threshold(self):
        held = force_balance(0, 4_000, 3_000, self.plant)
        self.assertEqual(held.acceleration, 0)
        moving = force_balance(0, 6_000, 3_000, self.plant)
        self.assertAlmostEqual(moving.acceleration, .05)

    def test_stop_event_prevents_backward_drift(self):
        state = TrainState(100, .1)
        step = forward_euler_step(state, -40_000, 0, 1, self.plant)
        self.assertEqual(step.state.speed, 0)
        self.assertGreaterEqual(step.state.position, 100)
        self.assertLess(step.state.position, 100.1)
        held = forward_euler_step(step.state, -40_000, 0, 1, self.plant)
        self.assertEqual(held.state, step.state)

    def test_exact_reference_starts_at_initial_state(self):
        state = TrainState(4, 10)
        self.assertEqual(exact_open_loop(0, state, 30_000, 0, self.plant), state)

    def test_validation(self):
        actions = (
            lambda: PlantParameters(mass=0), lambda: PlantParameters(brake_max=-1),
            lambda: TrainState(0, -1), lambda: force_balance(1, 0, -1, self.plant),
            lambda: forward_euler_step(TrainState(), 0, 0, 0, self.plant),
        )
        for action in actions:
            with self.assertRaises(ValueError):
                action()


if __name__ == "__main__":
    unittest.main()
