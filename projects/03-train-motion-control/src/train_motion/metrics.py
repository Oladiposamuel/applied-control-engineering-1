"""Performance metrics that distinguish crossing, settling, and stopping."""
import math
import numpy as np


def _crossing(run: dict) -> tuple[float | None, float | None]:
    target = run["controller"].position_target
    x, v, time = run["x_m"], run["v_mps"], run["time"]
    for index in range(len(x) - 1):
        if x[index] < target <= x[index + 1]:
            span = x[index + 1] - x[index]
            fraction = 0.0 if span == 0 else (target - x[index]) / span
            return (time[index] + fraction * (time[index + 1] - time[index]),
                    v[index] + fraction * (v[index + 1] - v[index]))
    return None, None


def _stop(run: dict, tolerance: float = 1e-8) -> tuple[float | None, float | None]:
    speed, position, time = run["v_mps"], run["x_m"], run["time"]
    for index in range(len(speed) - 1):
        if speed[index] > tolerance and speed[index + 1] <= tolerance:
            offset = run["stop_time_offset_s"][index]
            event_time = time[index] + (offset if offset >= 0 else run["dt"])
            return event_time, position[index + 1]
    return None, None


def _sustained_band(time, values, target, tolerance) -> float | None:
    inside = np.abs(values - target) <= tolerance
    for index in np.flatnonzero(inside):
        if np.all(inside[index:]):
            return float(time[index])
    return None


def compute_metrics(run: dict, speed_tolerance: float = 0.25,
                    position_tolerance: float = 1.0,
                    stopped_speed_tolerance: float = 0.05) -> dict:
    controller = run["controller"]
    speed_mode = controller.mode.startswith("speed_")
    position_mode = controller.mode.startswith("position_")
    crossing_time, crossing_speed = _crossing(run)
    stop_time, stop_position = _stop(run)
    tail = max(1, len(run["v_mps"]) // 5)
    speed_errors = controller.speed_target - run["v_mps"][-tail:]
    position_error = controller.position_target - run["x_m"][-1]
    stopped_at_target = (run["v_mps"][-1] <= stopped_speed_tolerance and
                         abs(position_error) <= position_tolerance)
    return {
        "controller_mode": controller.mode,
        "integral_units": run["integral_units"],
        "final_position_m": float(run["x_m"][-1]),
        "final_speed_mps": float(run["v_mps"][-1]),
        "final_position_error_m": float(position_error) if position_mode else None,
        "final_speed_error_mps": float(controller.speed_target - run["v_mps"][-1]) if speed_mode else None,
        "steady_window_mean_speed_error_mps": float(np.mean(speed_errors)) if speed_mode else None,
        "peak_position_overshoot_m": float(max(0.0, np.max(run["x_m"] - controller.position_target))) if position_mode else None,
        "station_crossing_time_s": None if crossing_time is None else float(crossing_time),
        "station_crossing_speed_mps": None if crossing_speed is None else float(crossing_speed),
        "first_stop_time_s": None if stop_time is None else float(stop_time),
        "first_stop_position_m": None if stop_position is None else float(stop_position),
        "stop_status": "stopped_at_target" if stopped_at_target else ("stopped_elsewhere" if stop_time is not None else "not_stopped"),
        "speed_settling_time_s": _sustained_band(run["time"], run["v_mps"], controller.speed_target, speed_tolerance) if speed_mode else None,
        "position_settling_time_s": _sustained_band(run["time"], run["x_m"], controller.position_target, position_tolerance) if position_mode else None,
        "max_abs_acceleration_mps2": float(np.max(np.abs(run["a_mps2"]))),
        "peak_requested_traction_N": float(max(0.0, np.max(run["force_requested_N"]))),
        "peak_requested_braking_N": float(max(0.0, -np.min(run["force_requested_N"]))),
        "peak_applied_traction_N": float(np.max(run["traction_applied_N"])),
        "peak_applied_braking_N": float(np.max(run["brake_applied_N"])),
        "saturated_steps": int(np.count_nonzero(run["is_saturated"])),
        "saturation_duration_s": float(np.count_nonzero(run["is_saturated"]) * run["dt"]),
        "anti_windup_activations": int(np.count_nonzero(run["anti_windup_active"])),
        "supervisor_override_steps": int(np.count_nonzero(run["supervisor_override"])),
        "supervisor_override_duration_s": float(np.count_nonzero(run["supervisor_override"]) * run["dt"]),
        "final_integral_state": float(run["z_integral"][-1]),
        "minimum_stopping_margin_m": float(np.min(run["stopping_margin_m"])),
        "all_values_finite": bool(all(np.all(np.isfinite(value)) for value in run.values() if isinstance(value, np.ndarray) and value.dtype != bool)),
    }
