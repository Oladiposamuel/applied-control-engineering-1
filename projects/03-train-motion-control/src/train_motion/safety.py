"""Educational stopping-feasibility estimates and brake supervision."""
from dataclasses import dataclass
import math

from .model import PlantParameters, TrainState, forward_euler_step


@dataclass(frozen=True)
class Feasibility:
    remaining_distance: float
    stopping_distance: float
    required_deceleration: float | None
    margin: float
    feasible: bool


def stopping_feasibility(position: float, speed: float, target: float,
                         available_deceleration: float, margin: float = 0.0,
                         response_delay: float = 0.0) -> Feasibility:
    values = (position, speed, target, available_deceleration, margin, response_delay)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Feasibility inputs must be finite")
    if speed < 0 or available_deceleration <= 0 or margin < 0 or response_delay < 0:
        raise ValueError("Speed/margins must be nonnegative and deceleration positive")
    remaining = target - position
    stopping = speed**2 / (2 * available_deceleration) + speed * response_delay
    required = speed**2 / (2 * remaining) if remaining > 0 else (0.0 if speed == 0 else None)
    feasible = speed == 0 or (remaining > 0 and remaining >= stopping + margin)
    return Feasibility(remaining, stopping, required, remaining - stopping, feasible)


def supervisor_force(state: TrainState, target: float, plant: PlantParameters,
                     available_deceleration: float, margin: float,
                     response_delay: float = 0.0) -> tuple[float | None, Feasibility]:
    check = stopping_feasibility(state.position, state.speed, target,
                                 available_deceleration, margin, response_delay)
    should_brake = state.speed > 0 and (state.position >= target or not check.feasible)
    return (-plant.brake_max if should_brake else None), check


def dynamic_stopping_distance(speed: float, plant: PlantParameters,
                              disturbance: float = 0.0, dt: float = 0.01,
                              horizon: float = 600.0) -> float:
    """Simulate the same plant under maximum active braking until stopped."""
    state = TrainState(0.0, speed)
    for _ in range(math.ceil(horizon / dt)):
        if state.speed <= 1e-10:
            return state.position
        state = forward_euler_step(state, -plant.brake_max, disturbance, dt, plant).state
    raise RuntimeError("Dynamic stopping simulation did not stop within horizon")
