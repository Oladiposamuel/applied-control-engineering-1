"""Signed open-loop, speed, and position controllers."""
from dataclasses import dataclass
import math

from .model import PlantParameters, TrainState, clip_force


MODES = frozenset({
    "open_loop", "speed_p", "speed_pi", "position_p", "position_pd",
    "position_pid", "position_pid_aw", "position_pid_supervisor",
})


@dataclass(frozen=True)
class ControllerParameters:
    mode: str = "speed_pi"
    fixed_force: float = 30_000.0
    speed_target: float = 25.0
    position_target: float = 500.0
    kp_speed: float = 1_000.0
    ki_speed: float = 100.0
    kp_position: float = 100.0
    ki_position: float = 5.0
    kd_position: float = 800.0
    nominal_disturbance: float = 0.0

    def __post_init__(self):
        numeric = tuple(value for key, value in self.__dict__.items() if key != "mode")
        if self.mode not in MODES:
            raise ValueError(f"mode must be one of {sorted(MODES)}")
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("Controller parameters must be finite")
        nonnegative = (self.speed_target, self.kp_speed, self.ki_speed,
                       self.kp_position, self.ki_position, self.kd_position,
                       self.nominal_disturbance)
        if min(nonnegative) < 0:
            raise ValueError("Targets, gains and nominal disturbance cannot be negative")


@dataclass(frozen=True)
class ControllerResult:
    speed_error: float | None
    position_error: float | None
    bias: float
    proportional: float
    integral: float
    derivative: float
    raw_force: float
    applied_force: float
    saturated: bool
    integral_rate: float
    anti_windup_active: bool
    integral_units: str | None


def uses_integral(mode: str) -> bool:
    return mode in {"speed_pi", "position_pid", "position_pid_aw", "position_pid_supervisor"}


def uses_anti_windup(mode: str) -> bool:
    return mode in {"speed_pi", "position_pid_aw", "position_pid_supervisor"}


def command(state: TrainState, integral_state: float,
            controller: ControllerParameters,
            plant: PlantParameters) -> ControllerResult:
    if not math.isfinite(integral_state):
        raise ValueError("Integral state must be finite")
    mode = controller.mode
    speed_error = controller.speed_target - state.speed if mode.startswith("speed_") else None
    position_error = controller.position_target - state.position if mode.startswith("position_") else None
    bias = proportional = integral = derivative = 0.0
    integral_rate = 0.0
    integral_units = None

    if mode == "open_loop":
        raw = controller.fixed_force
    elif mode.startswith("speed_"):
        bias = (plant.rolling_resistance + plant.velocity_resistance * controller.speed_target +
                controller.nominal_disturbance)
        proportional = controller.kp_speed * speed_error
        if mode == "speed_pi":
            integral = controller.ki_speed * integral_state
            integral_rate = speed_error
            integral_units = "m"
        raw = bias + proportional + integral
    else:
        proportional = controller.kp_position * position_error
        if mode in {"position_pd", "position_pid", "position_pid_aw", "position_pid_supervisor"}:
            derivative = -controller.kd_position * state.speed
        if uses_integral(mode):
            integral = controller.ki_position * integral_state
            integral_rate = position_error
            integral_units = "m*s"
        raw = proportional + integral + derivative

    applied, saturated = clip_force(raw, plant)
    anti_windup_active = False
    if uses_integral(mode):
        gain = controller.ki_speed if mode == "speed_pi" else controller.ki_position
        if gain == 0:
            integral_rate = 0.0
        elif uses_anti_windup(mode):
            pushes_high = raw >= plant.traction_max and integral_rate > 0
            pushes_low = raw <= -plant.brake_max and integral_rate < 0
            if pushes_high or pushes_low:
                integral_rate = 0.0
                anti_windup_active = True
    return ControllerResult(speed_error, position_error, bias, proportional,
                            integral, derivative, raw, applied, saturated,
                            integral_rate, anti_windup_active, integral_units)
