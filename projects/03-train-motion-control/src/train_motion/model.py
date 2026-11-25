"""Physically constrained one-dimensional train model.

The model is deliberately educational: level track, forward-only motion,
constant rolling resistance while moving, and a linear speed resistance.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlantParameters:
    mass: float = 20_000.0
    rolling_resistance: float = 2_000.0
    velocity_resistance: float = 500.0
    traction_max: float = 40_000.0
    brake_max: float = 40_000.0

    def __post_init__(self):
        values = (self.mass, self.rolling_resistance, self.velocity_resistance,
                  self.traction_max, self.brake_max)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Plant parameters must be finite")
        if self.mass <= 0:
            raise ValueError("Mass must be positive")
        if min(values[1:]) < 0:
            raise ValueError("Resistance and actuator limits cannot be negative")


@dataclass(frozen=True)
class TrainState:
    position: float = 0.0
    speed: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.position) or not math.isfinite(self.speed):
            raise ValueError("State values must be finite")
        if self.speed < 0:
            raise ValueError("Forward-only speed cannot be negative")


@dataclass(frozen=True)
class ForceResult:
    requested: float
    applied: float
    traction: float
    braking: float
    rolling: float
    velocity: float
    disturbance: float
    net: float
    acceleration: float
    saturated: bool
    standstill_constraint: bool


@dataclass(frozen=True)
class StepResult:
    state: TrainState
    forces: ForceResult
    stopped_within_step: bool
    stop_time_offset: float | None
    distance_advanced: float


def clip_force(requested: float, plant: PlantParameters) -> tuple[float, bool]:
    if not math.isfinite(requested):
        raise ValueError("Requested force must be finite")
    applied = min(plant.traction_max, max(-plant.brake_max, requested))
    return applied, not math.isclose(applied, requested, abs_tol=1e-12, rel_tol=0.0)


def force_balance(speed: float, requested: float, disturbance: float,
                  plant: PlantParameters) -> ForceResult:
    """Evaluate executed force and acceleration at one old-state sample.

    At standstill the track constraint prevents reverse movement. A positive
    request must exceed rolling resistance plus the opposing disturbance before
    forward acceleration begins; a braking request simply holds the train.
    """
    if speed < 0 or not math.isfinite(speed):
        raise ValueError("Speed must be finite and nonnegative")
    if disturbance < 0 or not math.isfinite(disturbance):
        raise ValueError("Default forward-only disturbance must be finite and nonnegative")
    applied, saturated = clip_force(requested, plant)
    traction = max(0.0, applied)
    braking = max(0.0, -applied)
    if speed <= 1e-12:
        threshold = plant.rolling_resistance + disturbance
        if applied <= threshold:
            return ForceResult(requested, applied, traction, braking, 0.0, 0.0,
                               disturbance, 0.0, 0.0, saturated, True)
        rolling = plant.rolling_resistance
        net = applied - rolling - disturbance
        return ForceResult(requested, applied, traction, braking, rolling, 0.0,
                           disturbance, net, net / plant.mass, saturated, False)
    rolling = plant.rolling_resistance
    velocity = plant.velocity_resistance * speed
    net = applied - rolling - velocity - disturbance
    return ForceResult(requested, applied, traction, braking, rolling, velocity,
                       disturbance, net, net / plant.mass, saturated, False)


def equilibrium_speed(applied_force: float, disturbance: float,
                      plant: PlantParameters) -> float | None:
    """Return the positive moving equilibrium, or None when no finite one exists."""
    available = applied_force - plant.rolling_resistance - disturbance
    if plant.velocity_resistance == 0:
        return 0.0 if math.isclose(available, 0.0, abs_tol=1e-12) else None
    return max(0.0, available / plant.velocity_resistance)


def exact_open_loop(t: float, state: TrainState, applied_force: float,
                    disturbance: float, plant: PlantParameters) -> TrainState:
    """Exact moving solution for constant force, c>0, with no stop event."""
    if t < 0 or plant.velocity_resistance <= 0 or state.speed <= 0:
        raise ValueError("Exact reference requires t>=0, c>0 and positive initial speed")
    terminal = ((applied_force - plant.rolling_resistance - disturbance) /
                plant.velocity_resistance)
    rate = plant.velocity_resistance / plant.mass
    decay = math.exp(-rate * t)
    speed = terminal + (state.speed - terminal) * decay
    position = (state.position + terminal * t +
                (state.speed - terminal) * (1.0 - decay) / rate)
    if speed <= 0:
        raise ValueError("Exact reference assumptions exclude a stop event")
    return TrainState(position, speed)


def _stop_event(state: TrainState, forces: ForceResult, disturbance: float,
                dt: float, plant: PlantParameters) -> tuple[float, float]:
    """Return event time and distance under constant actuator/disturbance."""
    base = forces.applied - plant.rolling_resistance - disturbance
    c = plant.velocity_resistance
    if c > 0:
        asymptote = base / c
        denominator = state.speed - asymptote
        ratio = -asymptote / denominator if denominator != 0 else -1.0
        if 0 < ratio <= 1:
            event_time = -math.log(ratio) * plant.mass / c
            rate = c / plant.mass
            distance = (asymptote * event_time +
                        (state.speed - asymptote) * (1.0 - math.exp(-rate * event_time)) / rate)
            return min(dt, max(0.0, event_time)), max(0.0, distance)
    acceleration = forces.acceleration
    event_time = min(dt, max(0.0, -state.speed / acceleration)) if acceleration < 0 else dt
    distance = state.speed * event_time + 0.5 * acceleration * event_time**2
    return event_time, max(0.0, distance)


def forward_euler_step(state: TrainState, requested_force: float,
                       disturbance: float, dt: float,
                       plant: PlantParameters) -> StepResult:
    """Advance both states from the same old sample with event-aware stopping."""
    if dt <= 0 or not math.isfinite(dt):
        raise ValueError("dt must be finite and positive")
    forces = force_balance(state.speed, requested_force, disturbance, plant)
    predicted_speed = state.speed + dt * forces.acceleration
    if state.speed > 0 and predicted_speed < 0:
        stop_time, distance = _stop_event(state, forces, disturbance, dt, plant)
        return StepResult(TrainState(state.position + distance, 0.0), forces,
                          True, stop_time, distance)
    next_speed = max(0.0, predicted_speed)
    distance = dt * state.speed
    return StepResult(TrainState(state.position + distance, next_speed), forces,
                      False, None, distance)
