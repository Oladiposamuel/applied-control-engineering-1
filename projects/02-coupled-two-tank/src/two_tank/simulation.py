"""Conservative explicit simulation of the coupled tanks."""
from dataclasses import dataclass
import math
import numpy as np

from .controllers import ControllerParameters, command, integral_derivative
from .model import TankParameters, requested_link_flow, requested_outlet_flow


@dataclass(frozen=True)
class Scenario:
    duration: float = 4000.0
    dt: float = 0.1
    disturbance_at: float | None = None
    demand: float = 0.0
    initial_integral: float = 0.0

    def __post_init__(self):
        values = (self.duration, self.dt, self.demand, self.initial_integral)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Scenario values must be finite")
        if self.duration <= 0 or self.dt <= 0 or self.demand < 0:
            raise ValueError("Duration/dt must be positive and demand nonnegative")
        if self.disturbance_at is not None and (
                not math.isfinite(self.disturbance_at) or
                not 0 <= self.disturbance_at <= self.duration):
            raise ValueError("Disturbance time outside simulation")
        steps = round(self.duration / self.dt)
        if not np.isclose(self.duration / self.dt, steps, atol=1e-8, rtol=0):
            raise ValueError("dt must divide duration")


def simulate(tank: TankParameters, controller: ControllerParameters,
             scenario: Scenario) -> dict:
    """Run a mass-conserving Euler-style flow-volume simulation.

    State arrays have N+1 samples; all flow and controller arrays have N
    samples aligned to ``time[:-1]``. Requested Tank 2 outlet and demand are
    proportionally scaled if their combined requested volume exceeds available
    Tank 2 inventory. This deterministic allocation preserves mass exactly.
    """
    n = round(scenario.duration / scenario.dt)
    dt = scenario.dt
    time = np.linspace(0, scenario.duration, n + 1)
    state_keys = ("h1", "h2", "integral_state", "total_volume")
    step_keys = ("pump_raw", "pump_actual", "q12_actual", "qout_actual",
                 "demand_requested", "demand_served", "demand_unmet",
                 "error_step", "volume_balance_residual")
    out = {key: np.empty(n + 1) for key in state_keys}
    out.update({key: np.empty(n) for key in step_keys})
    out["saturated"] = np.empty(n, dtype=bool)
    out["integrating"] = np.empty(n, dtype=bool)
    out["h1"][0], out["h2"][0] = tank.initial_height1, tank.initial_height2
    out["integral_state"][0] = scenario.initial_integral
    out["total_volume"][0] = tank.area1*out["h1"][0] + tank.area2*out["h2"][0]

    for i in range(n):
        h1, h2, z = out["h1"][i], out["h2"][i], out["integral_state"][i]
        error, raw, pump, saturated = command(h2, z, controller)
        z_dot, integrating = integral_derivative(error, raw, controller)
        demand = scenario.demand if (scenario.disturbance_at is not None and
                                     time[i] >= scenario.disturbance_at - 1e-9) else 0.0

        # Execute volumes from start-of-step requests, capped by inventory.
        v1_available = tank.area1*h1 + pump*dt
        transfer_volume = min(requested_link_flow(h1, h2, tank)*dt, v1_available)
        v1_next = v1_available - transfer_volume
        v2_available = tank.area2*h2 + transfer_volume
        outlet_request = requested_outlet_flow(h2, tank)*dt
        demand_request = demand*dt
        total_request = outlet_request + demand_request
        scale = min(1.0, v2_available/total_request) if total_request > 0 else 1.0
        outlet_volume = outlet_request*scale
        demand_volume = demand_request*scale
        v2_next = v2_available - outlet_volume - demand_volume

        out["h1"][i+1] = max(0.0, v1_next/tank.area1)
        out["h2"][i+1] = max(0.0, v2_next/tank.area2)
        out["integral_state"][i+1] = z + dt*z_dot
        out["pump_raw"][i], out["pump_actual"][i] = raw, pump
        out["q12_actual"][i] = transfer_volume/dt
        out["qout_actual"][i] = outlet_volume/dt
        out["demand_requested"][i] = demand
        out["demand_served"][i] = demand_volume/dt
        out["demand_unmet"][i] = demand - out["demand_served"][i]
        out["error_step"][i] = error
        out["saturated"][i], out["integrating"][i] = saturated, integrating
        out["total_volume"][i+1] = v1_next + v2_next
        expected_change = dt*(pump-out["qout_actual"][i]-out["demand_served"][i])
        out["volume_balance_residual"][i] = (
            out["total_volume"][i+1]-out["total_volume"][i]-expected_change)
        if min(v1_next, v2_next) < -1e-10:
            raise RuntimeError("Negative inventory violates physical constraints")

    out.update(time=time, dt=dt, duration=scenario.duration,
               controller_mode=controller.mode, tank_parameters=tank,
               controller_parameters=controller, scenario=scenario)
    return out
