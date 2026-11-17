"""Physical parameters and analytical reference calculations."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TankParameters:
    area1: float = 2.0
    area2: float = 1.5
    link_coefficient: float = 0.012
    outlet_coefficient: float = 0.008
    initial_height1: float = 2.0
    initial_height2: float = 0.5

    def __post_init__(self):
        values = (self.area1, self.area2, self.link_coefficient,
                  self.outlet_coefficient, self.initial_height1,
                  self.initial_height2)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Tank parameters must be finite")
        if min(self.area1, self.area2, self.link_coefficient,
               self.outlet_coefficient) <= 0:
            raise ValueError("Areas and flow coefficients must be positive")
        if min(self.initial_height1, self.initial_height2) < 0:
            raise ValueError("Initial heights cannot be negative")


def requested_link_flow(height1: float, height2: float,
                        params: TankParameters) -> float:
    """One-way requested flow from Tank 1 to Tank 2, in m³/s."""
    return max(0.0, params.link_coefficient * (height1 - height2))


def requested_outlet_flow(height2: float, params: TankParameters) -> float:
    return params.outlet_coefficient * height2


def open_loop_equilibrium(pump_flow: float,
                          params: TankParameters) -> tuple[float, float]:
    h2 = pump_flow / params.outlet_coefficient
    h1 = h2 + pump_flow / params.link_coefficient
    return h1, h2


def target_equilibrium(target: float, demand: float,
                       params: TankParameters) -> tuple[float, float, float]:
    pump = params.outlet_coefficient * target + demand
    height1 = target + pump / params.link_coefficient
    return height1, target, pump
