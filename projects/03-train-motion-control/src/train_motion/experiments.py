"""Reproducible Project 03 experiment suite and command-line entry point."""
from dataclasses import asdict, replace
import argparse
import json
from pathlib import Path

from .controllers import ControllerParameters
from .export import export_run
from .metrics import compute_metrics
from .model import PlantParameters, TrainState, exact_open_loop
from .plotting import plot_comparison, plot_forces_and_safety, plot_overview
from .simulation import Scenario, simulate
from .safety import stopping_feasibility


EXPERIMENTS = {
    "a_open_loop": "Open-loop traction and equilibrium",
    "b_braking": "Active braking and event-aware stop",
    "c_speed_p": "Biased speed P under an unknown resistance",
    "d_speed_pi": "Speed PI recovery compared with P",
    "e_position_pd": "Position P compared with velocity-damped PD",
    "f_position_pid": "Position PID and integral memory",
    "g_anti_windup": "Position PID windup and conditional integration",
    "h_brake_supervisor": "Stopping feasibility and brake supervision",
    "i_robustness": "Stronger disturbance and timestep robustness",
}


def _definition(name: str):
    baseline = PlantParameters()
    if name == "a_open_loop":
        return baseline, ControllerParameters(mode="open_loop", fixed_force=30_000), Scenario(duration=240, dt=.1, initial_speed=10), []
    if name == "b_braking":
        return baseline, ControllerParameters(mode="open_loop", fixed_force=-10_000), Scenario(duration=30, dt=.1, initial_position=100, initial_speed=20), []
    if name == "c_speed_p":
        c = ControllerParameters(mode="speed_p")
        return baseline, c, Scenario(duration=180, dt=.1, initial_speed=15, disturbance=3_000, disturbance_at=80), []
    if name == "d_speed_pi":
        c = ControllerParameters(mode="speed_pi")
        s = Scenario(duration=180, dt=.1, initial_speed=15, disturbance=3_000, disturbance_at=80)
        return baseline, c, s, [("Speed P", replace(c, mode="speed_p"))]
    if name == "e_position_pd":
        plant = replace(baseline, rolling_resistance=0, velocity_resistance=0)
        c = ControllerParameters(mode="position_pd")
        s = Scenario(duration=120, dt=.1, initial_position=400, initial_speed=20)
        return plant, c, s, [("Position P", replace(c, mode="position_p"))]
    if name == "f_position_pid":
        plant = replace(baseline, rolling_resistance=0, velocity_resistance=0)
        c = ControllerParameters(mode="position_pid")
        s = Scenario(duration=120, dt=.1, initial_position=400, initial_speed=20, initial_integral=200)
        return plant, c, s, [("Position PD", replace(c, mode="position_pd"))]
    if name == "g_anti_windup":
        c = ControllerParameters(mode="position_pid_aw")
        s = Scenario(duration=150, dt=.1, initial_position=0, initial_speed=0)
        return baseline, c, s, [("PID without anti-windup", replace(c, mode="position_pid"))]
    if name == "h_brake_supervisor":
        c = ControllerParameters(mode="position_pid_supervisor")
        s = Scenario(duration=60, dt=.05, initial_position=400, initial_speed=20, available_deceleration=1, safety_margin=0)
        return baseline, c, s, [("PID only", replace(c, mode="position_pid_aw"))]
    if name == "i_robustness":
        c = ControllerParameters(mode="speed_pi")
        s = Scenario(duration=180, dt=.05, initial_speed=15, disturbance=6_000, disturbance_at=60)
        return baseline, c, s, [("Speed P", replace(c, mode="speed_p"))]
    raise KeyError(name)


def _analysis(name: str, plant: PlantParameters, controller: ControllerParameters,
              scenario: Scenario, run: dict) -> dict:
    analysis = {}
    if name == "a_open_loop":
        exact = exact_open_loop(scenario.duration, TrainState(scenario.initial_position, scenario.initial_speed), controller.fixed_force, 0, plant)
        analysis.update(analytic_equilibrium_speed_mps=56.0,
                        final_exact_speed_mps=exact.speed,
                        final_euler_speed_error_mps=run["v_mps"][-1] - exact.speed,
                        final_euler_position_error_m=run["x_m"][-1] - exact.position)
        convergence = {}
        for dt in (1.0, .2, .05):
            candidate = simulate(plant, controller, replace(scenario, dt=dt))
            convergence[str(dt)] = {
                "speed_error_mps": float(candidate["v_mps"][-1] - exact.speed),
                "position_error_m": float(candidate["x_m"][-1] - exact.position),
            }
        analysis["euler_convergence"] = convergence
    elif name == "b_braking":
        convergence = {}
        for dt in (1.0, .2, .05):
            candidate = simulate(plant, controller, replace(scenario, dt=dt))
            candidate_metrics = compute_metrics(candidate)
            convergence[str(dt)] = {"stop_time_s": candidate_metrics["first_stop_time_s"],
                                    "stop_position_m": candidate_metrics["first_stop_position_m"]}
        analysis["stop_event_convergence"] = convergence
    elif name == "c_speed_p":
        analysis["analytic_disturbed_equilibrium_mps"] = 23.0
    elif name == "d_speed_pi":
        analysis.update(analytic_target_mps=25.0, analytic_integral_state_m=30.0,
                        required_force_N=17_500.0)
    elif name == "h_brake_supervisor":
        one = stopping_feasibility(400, 20, 500, 1)
        two = stopping_feasibility(400, 20, 500, 2)
        analysis["constant_deceleration_checks"] = {
            "1_mps2": asdict(one), "2_mps2": asdict(two),
            "constant_1_mps2_stop_position_m": 600.0,
        }
    elif name == "i_robustness":
        mild_scenario = replace(scenario, dt=.1, disturbance=3_000, disturbance_at=80)
        mild = simulate(plant, controller, mild_scenario)
        analysis["disturbance_cases"] = {
            "3000_N_at_80_s": {"final_speed_mps": float(mild["v_mps"][-1]),
                                "final_integral_m": float(mild["z_integral"][-1])},
            "6000_N_at_60_s": {"final_speed_mps": float(run["v_mps"][-1]),
                                "final_integral_m": float(run["z_integral"][-1])},
        }
        convergence = {}
        for dt in (.2, .1, .05):
            candidate = simulate(plant, controller, replace(scenario, dt=dt))
            convergence[str(dt)] = {"final_speed_mps": float(candidate["v_mps"][-1]),
                                    "final_integral_m": float(candidate["z_integral"][-1])}
        analysis["timestep_convergence"] = convergence
        limited = simulate(replace(plant, traction_max=16_000), controller, scenario)
        analysis["bounded_motor_case"] = {
            "traction_limit_N": 16_000,
            "saturated_steps": int(limited["is_saturated"].sum()),
            "final_speed_mps": float(limited["v_mps"][-1]),
            "final_integral_m": float(limited["z_integral"][-1]),
        }
    return analysis


def run_experiment(name: str, results_root: Path) -> tuple[dict, dict]:
    plant, controller, scenario, comparisons = _definition(name)
    run = simulate(plant, controller, scenario)
    metrics = compute_metrics(run)
    metrics["analysis"] = _analysis(name, plant, controller, scenario, run)
    directory = results_root / name
    export_run(directory, run, metrics, name, EXPERIMENTS[name])
    plot_overview(run, directory / "overview.png", EXPERIMENTS[name])
    plot_forces_and_safety(run, directory / "forces.png", EXPERIMENTS[name])
    compared = [(controller.mode.replace("_", " ").title(), run)]
    for label, alternative in comparisons:
        compared.append((label, simulate(plant, alternative, scenario)))
    if len(compared) > 1:
        plot_comparison(compared, directory / "comparison.png", EXPERIMENTS[name])
    return run, metrics


def write_frontend_fixture(path: Path) -> dict:
    plant = PlantParameters()
    controller = ControllerParameters(mode="speed_pi")
    scenario = Scenario(duration=20, dt=.1, initial_position=0, initial_speed=15,
                        disturbance=3_000, disturbance_at=10)
    run = simulate(plant, controller, scenario)
    indices = (0, 1, 50, 100, 150, 199)
    samples = []
    for index in indices:
        samples.append({
            "index": index, "t": float(run["time"][index]),
            "x_m": float(run["x_m"][index]), "v_mps": float(run["v_mps"][index]),
            "z_integral": float(run["z_integral"][index]),
            "force_requested_N": float(run["force_requested_N"][index]),
            "force_applied_N": float(run["force_applied_N"][index]),
            "a_mps2": float(run["a_mps2"][index]),
        })
    fixture = {"tolerance": 1e-8, "plant": asdict(plant),
               "controller": asdict(controller), "scenario": asdict(scenario),
               "samples": samples}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(fixture, indent=2, sort_keys=True) + "\n")
    return fixture


def run_experiments(results_root: Path | None = None,
                    selected: list[str] | None = None) -> dict:
    root = results_root or Path(__file__).resolve().parents[2] / "results"
    root.mkdir(parents=True, exist_ok=True)
    names = selected or list(EXPERIMENTS)
    records = {}
    for name in names:
        if name not in EXPERIMENTS:
            raise ValueError(f"Unknown experiment {name!r}")
        run, metrics = run_experiment(name, root)
        records[name] = metrics
        print(f"{name}: final x={run['x_m'][-1]:.3f} m, v={run['v_mps'][-1]:.3f} m/s, stop={metrics['stop_status']}")
    summary = {name: records[name] for name in names}
    (root / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return records


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--all", action="store_true", help="run all experiments (default)")
    parser.add_argument("--experiment", action="append", choices=sorted(EXPERIMENTS), help="run one named experiment; repeatable")
    parser.add_argument("--results", type=Path, help="override results directory")
    parser.add_argument("--frontend-fixture", type=Path, help="also write the canonical browser-parity fixture")
    args = parser.parse_args()
    run_experiments(args.results, args.experiment)
    if args.frontend_fixture:
        write_frontend_fixture(args.frontend_fixture)
        print(f"Frontend fixture: {args.frontend_fixture}")


if __name__ == "__main__":
    main()
