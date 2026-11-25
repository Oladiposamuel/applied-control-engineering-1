import unittest
from dataclasses import replace

from train_motion.controllers import ControllerParameters, command
from train_motion.model import (PlantParameters, TrainState, equilibrium_speed,
                                force_balance, forward_euler_step)
from train_motion.safety import stopping_feasibility


class HandwrittenRegressionTests(unittest.TestCase):
    def setUp(self):
        self.plant = PlantParameters()

    def test_initial_traction_and_euler(self):
        force = force_balance(10, 30_000, 0, self.plant)
        self.assertEqual(force.net, 23_000)
        self.assertAlmostEqual(force.acceleration, 1.15)
        step = forward_euler_step(TrainState(0, 10), 30_000, 0, 2, self.plant)
        self.assertAlmostEqual(step.state.position, 20)
        self.assertAlmostEqual(step.state.speed, 12.3)

    def test_open_loop_equilibrium_and_above_equilibrium(self):
        self.assertAlmostEqual(equilibrium_speed(30_000, 0, self.plant), 56)
        force = force_balance(60, 30_000, 0, self.plant)
        self.assertEqual(force.rolling + force.velocity, 32_000)
        self.assertEqual(force.net, -2_000)
        self.assertAlmostEqual(force.acceleration, -.1)

    def test_braking_initial_and_three_euler_steps(self):
        force = force_balance(20, -10_000, 0, self.plant)
        self.assertEqual(force.net, -22_000)
        self.assertAlmostEqual(force.acceleration, -1.1)
        state = TrainState(0, 20)
        expected = ((20, 18.9), (38.9, 17.8275), (56.7275, 16.7818125))
        for position, speed in expected:
            state = forward_euler_step(state, -10_000, 0, 1, self.plant).state
            self.assertAlmostEqual(state.position, position)
            self.assertAlmostEqual(state.speed, speed)
        first = forward_euler_step(TrainState(100, 20), -10_000, 0, 1, self.plant)
        self.assertAlmostEqual(first.state.position, 120)
        self.assertAlmostEqual(first.state.speed, 18.9)

    def test_speed_p_snapshots_and_disturbed_equilibrium(self):
        p = ControllerParameters(mode="speed_p")
        nominal = command(TrainState(0, 15), 0, p, self.plant)
        self.assertEqual(nominal.bias, 14_500)
        self.assertEqual(nominal.raw_force, 24_500)
        disturbed = force_balance(25, 14_500, 3_000, self.plant)
        self.assertEqual(disturbed.net, -3_000)
        self.assertAlmostEqual(disturbed.acceleration, -.15)
        equilibrium = command(TrainState(0, 23), 0, p, self.plant)
        self.assertEqual(equilibrium.applied_force, 16_500)
        self.assertEqual(force_balance(23, equilibrium.applied_force, 3_000, self.plant).net, 0)

    def test_speed_pi_disturbed_target_snapshot(self):
        pi = ControllerParameters(mode="speed_pi")
        result = command(TrainState(0, 25), 30, pi, self.plant)
        self.assertEqual(result.raw_force, 17_500)
        self.assertEqual(force_balance(25, result.applied_force, 3_000, self.plant).net, 0)

    def test_position_controller_snapshots_in_ideal_model(self):
        plant = replace(self.plant, rolling_resistance=0, velocity_resistance=0)
        state = TrainState(400, 20)
        p = command(state, 0, ControllerParameters(mode="position_p"), plant)
        pd = command(state, 0, ControllerParameters(mode="position_pd"), plant)
        pid = command(state, 200, ControllerParameters(mode="position_pid"), plant)
        self.assertEqual(p.raw_force, 10_000)
        self.assertAlmostEqual(force_balance(20, p.raw_force, 0, plant).acceleration, .5)
        self.assertEqual(pd.raw_force, -6_000)
        self.assertAlmostEqual(force_balance(20, pd.raw_force, 0, plant).acceleration, -.3)
        self.assertEqual(pid.raw_force, -5_000)
        self.assertAlmostEqual(force_balance(20, pid.raw_force, 0, plant).acceleration, -.25)
        crossing = command(TrainState(500, 5), 200, ControllerParameters(mode="position_pid"), plant)
        self.assertEqual(crossing.raw_force, -3_000)

    def test_pid_saturation_and_unrestricted_integrator_hand_check(self):
        result = command(TrainState(0, 0), 0, ControllerParameters(mode="position_pid"), self.plant)
        self.assertEqual(result.raw_force, 50_000)
        self.assertEqual(result.applied_force, 40_000)
        z_next = 0 + 1 * (500 - 0)
        self.assertEqual(z_next, 500)
        self.assertEqual(ControllerParameters().ki_position * z_next, 2_500)

    def test_constant_deceleration_stop_check(self):
        check = stopping_feasibility(400, 20, 500, 1)
        self.assertEqual(check.remaining_distance, 100)
        self.assertEqual(check.stopping_distance, 200)
        self.assertEqual(check.required_deceleration, 2)
        self.assertFalse(check.feasible)
        self.assertEqual(400 + check.stopping_distance, 600)


if __name__ == "__main__":
    unittest.main()
