import unittest

from train_motion.model import PlantParameters, TrainState
from train_motion.safety import (dynamic_stopping_distance, stopping_feasibility,
                                 supervisor_force)


class SafetyTests(unittest.TestCase):
    def test_feasible_and_infeasible_edges(self):
        self.assertFalse(stopping_feasibility(400, 20, 500, 1).feasible)
        self.assertTrue(stopping_feasibility(400, 20, 500, 2).feasible)
        self.assertFalse(stopping_feasibility(500, 1, 500, 2).feasible)
        self.assertTrue(stopping_feasibility(500, 0, 500, 2).feasible)

    def test_response_delay_increases_stop_distance(self):
        base = stopping_feasibility(0, 10, 100, 2)
        delayed = stopping_feasibility(0, 10, 100, 2, response_delay=1)
        self.assertEqual(delayed.stopping_distance - base.stopping_distance, 10)

    def test_supervisor_override_is_separate(self):
        plant = PlantParameters()
        force, check = supervisor_force(TrainState(400, 20), 500, plant, 1, 0)
        self.assertEqual(force, -plant.brake_max)
        self.assertFalse(check.feasible)
        force, check = supervisor_force(TrainState(0, 5), 500, plant, 2, 0)
        self.assertIsNone(force)
        self.assertTrue(check.feasible)

    def test_dynamic_stop_is_positive_and_finite(self):
        distance = dynamic_stopping_distance(20, PlantParameters(), dt=.02)
        self.assertGreater(distance, 0)
        self.assertLess(distance, 200)


if __name__ == "__main__":
    unittest.main()
