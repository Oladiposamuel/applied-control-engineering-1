"""Deterministic old-state Euler simulation with event-aware stopping."""
from dataclasses import dataclass
import math
import numpy as np

from .controllers import ControllerParameters, command
from .model import PlantParameters, TrainState, forward_euler_step
from .safety import stopping_feasibility, supervisor_force


@dataclass(frozen=True)
class Scenario:
    duration: float = 180.0
    dt: float = 0.1
    initial_position: float = 0.0
    initial_speed: float = 0.0
    initial_integral: float = 0.0
    disturbance: float = 0.0
    disturbance_at: float | None = None
    available_deceleration: float = 2.0
    safety_margin: float = 5.0
    response_delay: float = 0.0

    def __post_init__(self):
        values = tuple(value for value in self.__dict__.values() if value is not None)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Scenario values must be finite")
        if self.duration <= 0 or self.dt <= 0:
            raise ValueError("Duration and dt must be positive")
        if self.initial_speed < 0 or self.disturbance < 0:
            raise ValueError("Initial speed and disturbance must be nonnegative")
        if self.available_deceleration <= 0 or min(self.safety_margin, self.response_delay) < 0:
            raise ValueError("Safety values are invalid")
        if self.disturbance_at is not None and not 0 <= self.disturbance_at <= self.duration:
            raise ValueError("Disturbance time lies outside the simulation")
        steps = round(self.duration / self.dt)
        if not math.isclose(self.duration / self.dt, steps, abs_tol=1e-9, rel_tol=0.0):
            raise ValueError("dt must divide duration")


STATE_KEYS = ("x_m", "v_mps", "z_integral")
STEP_KEYS = (
    "a_mps2", "x_ref_m", "v_ref_mps", "e_x_m", "e_v_mps",
    "force_requested_N", "force_applied_N", "traction_applied_N",
    "brake_applied_N", "rolling_resistance_N", "velocity_resistance_N",
    "disturbance_N", "net_force_N", "remaining_distance_m",
    "estimated_stopping_distance_m", "stopping_margin_m",
)


def simulate(plant: PlantParameters, controller: ControllerParameters,
             scenario: Scenario) -> dict:
    n = round(scenario.duration / scenario.dt)
    time = np.linspace(0.0, scenario.duration, n + 1)
    out = {key: np.empty(n + 1) for key in STATE_KEYS}
    out.update({key: np.empty(n) for key in STEP_KEYS})
    for key in ("is_saturated", "anti_windup_active", "feasible_stop",
                "supervisor_override", "stopped", "stop_event"):
        out[key] = np.zeros(n, dtype=bool)
    # -1 is the explicit "no event in this step" sentinel; exported numerical
    # arrays remain finite rather than hiding absence behind NaN.
    out["stop_time_offset_s"] = np.full(n, -1.0)
    out["x_m"][0], out["v_mps"][0] = scenario.initial_position, scenario.initial_speed
    out["z_integral"][0] = scenario.initial_integral
    supervisor_latched = False

    for index in range(n):
        state = TrainState(out["x_m"][index], out["v_mps"][index])
        integral = out["z_integral"][index]
        result = command(state, integral, controller, plant)
        disturbance = (scenario.disturbance if scenario.disturbance_at is not None and
                       time[index] >= scenario.disturbance_at - 1e-12 else 0.0)
        request = result.raw_force
        override = False
        feasibility = stopping_feasibility(
            state.position, state.speed, controller.position_target,
            scenario.available_deceleration, scenario.safety_margin,
            scenario.response_delay)
        if controller.mode == "position_pid_supervisor":
            if state.speed <= 1e-10:
                supervisor_latched = False
            # Once stationary inside the documented 0.5 m hold band, command
            # zero rather than repeatedly nudging and re-braking at the target.
            hold_at_target = (state.speed <= 1e-10 and
                              abs(controller.position_target - state.position) <= 0.5)
            if hold_at_target:
                request, override = 0.0, True
            elif not supervisor_latched:
                supervised, feasibility = supervisor_force(
                    state, controller.position_target, plant,
                    scenario.available_deceleration, scenario.safety_margin,
                    scenario.response_delay)
                supervisor_latched = supervised is not None
            if supervisor_latched and not hold_at_target:
                request, override = -plant.brake_max, True

        step = forward_euler_step(state, request, disturbance, scenario.dt, plant)
        integral_rate = 0.0 if override else result.integral_rate
        out["x_m"][index + 1] = step.state.position
        out["v_mps"][index + 1] = step.state.speed
        out["z_integral"][index + 1] = integral + scenario.dt * integral_rate

        force = step.forces
        values = {
            "a_mps2": force.acceleration, "x_ref_m": controller.position_target,
            "v_ref_mps": controller.speed_target,
            "e_x_m": controller.position_target - state.position,
            "e_v_mps": controller.speed_target - state.speed,
            "force_requested_N": request, "force_applied_N": force.applied,
            "traction_applied_N": force.traction, "brake_applied_N": force.braking,
            "rolling_resistance_N": force.rolling,
            "velocity_resistance_N": force.velocity,
            "disturbance_N": disturbance, "net_force_N": force.net,
            "remaining_distance_m": feasibility.remaining_distance,
            "estimated_stopping_distance_m": feasibility.stopping_distance,
            "stopping_margin_m": feasibility.margin,
        }
        for key, value in values.items():
            out[key][index] = value
        out["is_saturated"][index] = force.saturated
        out["anti_windup_active"][index] = result.anti_windup_active or override
        out["feasible_stop"][index] = feasibility.feasible
        out["supervisor_override"][index] = override
        out["stopped"][index] = step.state.speed <= 1e-10
        out["stop_event"][index] = step.stopped_within_step
        if step.stop_time_offset is not None:
            out["stop_time_offset_s"][index] = step.stop_time_offset

    if not all(np.all(np.isfinite(out[key])) for key in STATE_KEYS):
        raise RuntimeError("Non-finite state generated")
    if np.any(out["v_mps"] < 0) or np.any(np.diff(out["x_m"]) < -1e-12):
        raise RuntimeError("Forward-only physical constraint violated")
    out.update(time=time, dt=scenario.dt, plant=plant, controller=controller,
               scenario=scenario, integral_units=("m" if controller.mode == "speed_pi"
               else "m*s" if controller.mode.startswith("position_pid") else None))
    return out
