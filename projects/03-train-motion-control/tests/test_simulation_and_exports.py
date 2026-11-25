import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

from train_motion.controllers import ControllerParameters
from train_motion.experiments import run_experiment, write_frontend_fixture
from train_motion.metrics import compute_metrics
from train_motion.model import PlantParameters
from train_motion.simulation import Scenario, simulate


class SimulationAndExportTests(unittest.TestCase):
    def test_old_state_update_and_alignment(self):
        run = simulate(PlantParameters(), ControllerParameters(mode="open_loop", fixed_force=30_000), Scenario(duration=2, dt=2, initial_speed=10))
        self.assertAlmostEqual(run["x_m"][1], 20)
        self.assertAlmostEqual(run["v_mps"][1], 12.3)
        self.assertEqual(len(run["time"]), 2)
        self.assertEqual(len(run["a_mps2"]), 1)

    def test_forward_only_invariants_and_repeatability(self):
        args = (PlantParameters(), ControllerParameters(mode="position_pid_aw"), Scenario(duration=20, dt=.1))
        first, second = simulate(*args), simulate(*args)
        self.assertTrue(np.all(first["v_mps"] >= 0))
        self.assertTrue(np.all(np.diff(first["x_m"]) >= 0))
        for key in ("x_m", "v_mps", "z_integral", "force_applied_N"):
            np.testing.assert_array_equal(first[key], second[key])

    def test_disturbance_switch(self):
        run = simulate(PlantParameters(), ControllerParameters(mode="speed_p"), Scenario(duration=2, dt=.1, initial_speed=15, disturbance=3_000, disturbance_at=1))
        np.testing.assert_array_equal(run["disturbance_N"][:10], 0)
        np.testing.assert_array_equal(run["disturbance_N"][10:], 3_000)

    def test_metrics_do_not_call_crossing_a_stop(self):
        run = simulate(PlantParameters(), ControllerParameters(mode="open_loop", fixed_force=30_000, position_target=10), Scenario(duration=2, dt=.1, initial_speed=10))
        metrics = compute_metrics(run)
        self.assertIsNotNone(metrics["station_crossing_time_s"])
        self.assertGreater(metrics["station_crossing_speed_mps"], 0)
        self.assertEqual(metrics["stop_status"], "not_stopped")

    def test_export_contract_and_fixture(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run_experiment("c_speed_p", root)
            folder = root / "c_speed_p"
            for name in ("config.json", "metrics.json", "timeseries.csv", "overview.png", "forces.png"):
                self.assertGreater((folder / name).stat().st_size, 20)
            config = json.loads((folder / "config.json").read_text())
            self.assertEqual(config["controller"]["mode"], "speed_p")
            fixture_path = root / "fixture.json"
            fixture = write_frontend_fixture(fixture_path)
            self.assertEqual(json.loads(fixture_path.read_text()), fixture)
            self.assertEqual(len(fixture["samples"]), 6)

    def test_validation(self):
        for action in (lambda: Scenario(dt=0), lambda: Scenario(duration=1, dt=.3), lambda: Scenario(initial_speed=-1)):
            with self.assertRaises(ValueError):
                action()


if __name__ == "__main__":
    unittest.main()
