"""Open-loop, proportional, and PI pump commands for Tank 2 regulation."""
from dataclasses import dataclass
import math
import numpy as np


MODES = frozenset({"open_loop", "p", "pi"})
ANTI_WINDUP = frozenset({"none", "conditional"})


@dataclass(frozen=True)
class ControllerParameters:
    mode: str = "pi"
    target: float = 3.0
    kp: float = 0.02
    ki: float = 0.0001
    bias: float = 0.024
    min_flow: float = 0.0
    max_flow: float = 0.10
    fixed_flow: float = 0.04
    anti_windup: str = "conditional"

    def __post_init__(self):
        numeric = (self.target, self.kp, self.ki, self.bias, self.min_flow,
                   self.max_flow, self.fixed_flow)
        if not all(math.isfinite(v) for v in numeric):
            raise ValueError("Controller parameters must be finite")
        if self.mode not in MODES:
            raise ValueError(f"mode must be one of {sorted(MODES)}")
        if self.anti_windup not in ANTI_WINDUP:
            raise ValueError(f"anti_windup must be one of {sorted(ANTI_WINDUP)}")
        if min(self.target, self.kp, self.ki, self.bias,
               self.min_flow, self.fixed_flow) < 0:
            raise ValueError("Controller values cannot be negative")
        if self.max_flow <= self.min_flow:
            raise ValueError("Maximum pump flow must exceed minimum")


def command(height2: float, integral_state: float,
            c: ControllerParameters) -> tuple[float, float, float, bool]:
    """Return error, raw command, actual command, and saturation flag."""
    error = c.target - height2
    if c.mode == "open_loop":
        raw = c.fixed_flow
    else:
        raw = c.bias + c.kp * error + (c.ki * integral_state if c.mode == "pi" else 0.0)
    actual = float(np.clip(raw, c.min_flow, c.max_flow))
    return error, raw, actual, not np.isclose(raw, actual, atol=1e-12, rtol=0)


def integral_derivative(error: float, raw: float,
                        c: ControllerParameters) -> tuple[float, bool]:
    if c.mode != "pi":
        return 0.0, False
    blocked = c.anti_windup == "conditional" and (
        (raw >= c.max_flow and error > 0) or
        (raw <= c.min_flow and error < 0)
    )
    return (0.0, False) if blocked else (error, True)
