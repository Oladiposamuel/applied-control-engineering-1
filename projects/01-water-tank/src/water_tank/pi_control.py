"""Phase 3: proportional-integral water-level control with anti-windup.

Run from the workspace root:
    PYTHONPATH=projects/01-water-tank/src python -m water_tank.pi_control

The controller and plant use a zero-order-held command and simultaneous
Forward Euler state updates. At step n, both h[n+1] and z[n+1] are calculated
only from h[n], z[n], and inputs applicable during that step.
"""
from dataclasses import dataclass
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .feedback_control import ControllerParameters, settling_time, simulate
from .tank_simulation import TankParameters


ANTI_WINDUP_STRATEGIES = frozenset({"none", "conditional"})
SATURATION_TOLERANCE = 1e-12


@dataclass(frozen=True)
class PIControllerParameters:
    """PI controller configuration with SI units.

    kp is in m²/s, ki in m/s², target in m, bias and pump limits in
    m³/s. ``anti_windup='conditional'`` freezes integration only when the
    current error would drive an already-saturated command farther outward.
    """

    kp: float = 0.02
    ki: float = 0.001
    target: float = 3.0
    bias: float = 0.035
    min_flow: float = 0.0
    max_flow: float = 0.10
    anti_windup: str = "conditional"

    def __post_init__(self):
        numeric = (self.kp, self.ki, self.target, self.bias,
                   self.min_flow, self.max_flow)
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("PI controller parameters must be finite")
        if self.kp < 0 or self.ki < 0:
            raise ValueError("PI gains must be nonnegative")
        if self.target < 0 or self.bias < 0 or self.min_flow < 0:
            raise ValueError("Target, bias, and minimum flow must be nonnegative")
        if self.max_flow <= self.min_flow:
            raise ValueError("Maximum flow must exceed minimum flow")
        if self.anti_windup not in ANTI_WINDUP_STRATEGIES:
            raise ValueError(f"anti_windup must be one of {sorted(ANTI_WINDUP_STRATEGIES)}")


def pi_command(height: float, integral_state: float,
               control: PIControllerParameters) -> tuple[float, float, bool]:
    """Return ``(raw_flow, actual_flow, saturated)`` for the current state.

    Height is in m, integral_state in m·s, and flows in m³/s. When Ki is
    zero, the integral state is multiplied by zero and cannot affect flow.
    """
    if not math.isfinite(height) or not math.isfinite(integral_state):
        raise ValueError("Height and integral state must be finite")
    error = control.target - height
    raw_flow = control.bias + control.kp * error + control.ki * integral_state
    actual_flow = float(np.clip(raw_flow, control.min_flow, control.max_flow))
    saturated = (raw_flow > control.max_flow + SATURATION_TOLERANCE or
                 raw_flow < control.min_flow - SATURATION_TOLERANCE)
    return raw_flow, actual_flow, saturated


def integral_rate(error: float, raw_flow: float,
                  control: PIControllerParameters) -> float:
    """Return dz/dt in m according to the configured anti-windup rule."""
    upper_blocked = (raw_flow > control.max_flow + SATURATION_TOLERANCE and
                     error > 0)
    lower_blocked = (raw_flow < control.min_flow - SATURATION_TOLERANCE and
                     error < 0)
    if control.anti_windup == "conditional" and (upper_blocked or lower_blocked):
        return 0.0
    return error


def simulate_pi(p: TankParameters, control: PIControllerParameters, *,
                duration: float = 1200.0, dt: float = 0.1,
                disturbance_at: float | None = None,
                extra_leak: float = 0.0,
                initial_integral: float = 0.0) -> dict[str, np.ndarray]:
    """Simulate the PI-controlled tank using simultaneous Forward Euler.

    ``time``, ``height``, ``integral_state``, and ``error`` contain N+1 state
    samples. ``raw_flow``, ``flow``, ``leak``, and ``saturated`` contain N
    per-step values, aligned with ``time[:-1]``. The returned ``steps`` is N.
    The integral state is still accumulated when Ki=0 for observability, but
    it cannot influence the command because its coefficient is zero.
    """
    numeric = (duration, dt, extra_leak, initial_integral)
    if not all(math.isfinite(value) for value in numeric):
        raise ValueError("Simulation inputs must be finite")
    if duration <= 0 or dt <= 0:
        raise ValueError("Duration and dt must be positive")
    if extra_leak < 0:
        raise ValueError("Extra leakage must be nonnegative")
    if disturbance_at is not None:
        if not math.isfinite(disturbance_at) or not 0 <= disturbance_at <= duration:
            raise ValueError("Disturbance time outside simulation interval")
    n_float = duration / dt
    steps = round(n_float)
    if not np.isclose(n_float, steps, atol=1e-8, rtol=0):
        raise ValueError("dt must divide duration")

    time = np.linspace(0.0, duration, steps + 1)
    height = np.empty(steps + 1)
    integral_state = np.empty(steps + 1)
    error = np.empty(steps + 1)
    raw_flow = np.empty(steps)
    flow = np.empty(steps)
    leak = np.empty(steps)
    saturated = np.empty(steps, dtype=bool)
    height[0] = p.initial_height
    integral_state[0] = initial_integral

    for i in range(steps):
        error[i] = control.target - height[i]
        raw_flow[i], flow[i], saturated[i] = pi_command(
            height[i], integral_state[i], control)
        disturbance_active = (disturbance_at is not None and
                              time[i] >= disturbance_at - 1e-9)
        leak[i] = p.leak_flow + (extra_leak if disturbance_active else 0.0)
        z_dot = integral_rate(error[i], raw_flow[i], control)
        h_dot = (flow[i] - p.outlet_coefficient * height[i] - leak[i]) / p.area
        # Both next states use only current state values: explicit Euler.
        height[i + 1] = height[i] + dt * h_dot
        integral_state[i + 1] = integral_state[i] + dt * z_dot
    error[-1] = control.target - height[-1]

    return {
        "time": time,
        "height": height,
        "integral_state": integral_state,
        "error": error,
        "raw_flow": raw_flow,
        "flow": flow,
        "leak": leak,
        "saturated": saturated,
        "steps": steps,
        "dt": dt,
        "duration": duration,
    }


def _decorate(axes, target: float, disturbance_at: float | None = None):
    axes[0].axhline(target, color="black", linestyle="--", linewidth=1,
                    label=f"Reference ({target:g} m)")
    for axis in axes:
        if disturbance_at is not None:
            axis.axvline(disturbance_at, color="black", linestyle=":",
                         alpha=0.65, label="Leak step")
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8)


def make_reference_tracking_plot(results: Path) -> tuple[dict, dict]:
    """Scenario A: compare equally tuned biased P and PI controllers."""
    plant = TankParameters(initial_height=1.0)
    pi = PIControllerParameters()
    p_only = ControllerParameters(kp=pi.kp, target=pi.target, bias=pi.bias,
                                  min_flow=pi.min_flow, max_flow=pi.max_flow)
    p_run = simulate(plant, p_only, duration=1200.0, dt=0.1)
    pi_run = simulate_pi(plant, pi, duration=1200.0, dt=0.1)
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True,
                             gridspec_kw={"height_ratios": [2, 1]})
    axes[0].plot(p_run["time"], p_run["height"], label="Biased P", linewidth=1.7)
    axes[0].plot(pi_run["time"], pi_run["height"], label="PI + anti-windup", linewidth=1.7)
    axes[1].step(p_run["time"][:-1], p_run["flow"], where="post", label="P flow")
    axes[1].step(pi_run["time"][:-1], pi_run["flow"], where="post", label="PI flow")
    axes[0].set(ylabel="Water height (m)", title="Phase 3A — Nominal reference tracking")
    axes[1].set(xlabel="Time (s)", ylabel="Inlet flow (m³/s)")
    _decorate(axes, pi.target)
    fig.tight_layout()
    fig.savefig(results / "phase3_pi_reference_tracking.png", dpi=170)
    plt.close(fig)
    return p_run, pi_run


def make_disturbance_plot(results: Path) -> tuple[dict, dict]:
    """Scenario B: show that PI removes the P-only disturbance offset."""
    plant = TankParameters(initial_height=3.0)
    pi = PIControllerParameters()
    p_only = ControllerParameters(kp=pi.kp, target=pi.target, bias=pi.bias,
                                  min_flow=pi.min_flow, max_flow=pi.max_flow)
    disturbance_at, extra_leak, duration = 600.0, 0.007, 2400.0
    p_run = simulate(plant, p_only, duration=duration, dt=0.1,
                     disturbance_at=disturbance_at, extra_leak=extra_leak)
    pi_run = simulate_pi(plant, pi, duration=duration, dt=0.1,
                         disturbance_at=disturbance_at, extra_leak=extra_leak)
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True,
                             gridspec_kw={"height_ratios": [2, 1, 1]})
    axes[0].plot(p_run["time"], p_run["height"], label="Biased P", linewidth=1.7)
    axes[0].plot(pi_run["time"], pi_run["height"], label="PI + anti-windup", linewidth=1.7)
    axes[1].step(p_run["time"][:-1], p_run["flow"], where="post", label="P flow")
    axes[1].step(pi_run["time"][:-1], pi_run["flow"], where="post", label="PI flow")
    axes[2].plot(pi_run["time"], pi_run["integral_state"], color="tab:green",
                 label="PI integral state z")
    axes[0].set(ylabel="Water height (m)", title="Phase 3B — Persistent leakage disturbance")
    axes[1].set(ylabel="Inlet flow (m³/s)")
    axes[2].set(xlabel="Time (s)", ylabel="z (m·s)")
    _decorate(axes, pi.target, disturbance_at)
    fig.tight_layout()
    fig.savefig(results / "phase3_pi_disturbance_rejection.png", dpi=170)
    plt.close(fig)
    return p_run, pi_run


def make_anti_windup_plot(results: Path) -> tuple[dict, dict]:
    """Scenario C: feasible low-pump case comparing integrator policies."""
    plant = TankParameters(initial_height=0.0)
    base = dict(kp=0.02, ki=0.001, target=3.0, bias=0.035,
                min_flow=0.0, max_flow=0.05)
    no_aw = PIControllerParameters(**base, anti_windup="none")
    conditional = PIControllerParameters(**base, anti_windup="conditional")
    no_aw_run = simulate_pi(plant, no_aw, duration=1800.0, dt=0.1)
    aw_run = simulate_pi(plant, conditional, duration=1800.0, dt=0.1)
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for run, label, color in ((no_aw_run, "No anti-windup", "tab:red"),
                              (aw_run, "Conditional anti-windup", "tab:blue")):
        axes[0].plot(run["time"], run["height"], label=label, color=color)
        axes[1].plot(run["time"], run["integral_state"], label=label, color=color)
        axes[2].step(run["time"][:-1], run["flow"], where="post",
                     label=f"Actual — {label}", color=color)
    axes[2].plot(no_aw_run["time"][:-1], no_aw_run["raw_flow"], color="tab:red",
                 alpha=0.45, linestyle="--", label="Raw — no anti-windup")
    axes[2].axhline(conditional.max_flow, color="black", linestyle=":",
                    label="Pump maximum (0.05 m³/s)")
    axes[0].set(ylabel="Water height (m)", title="Phase 3C — Integral windup and recovery")
    axes[1].set(ylabel="z (m·s)")
    axes[2].set(xlabel="Time (s)", ylabel="Inlet flow (m³/s)")
    _decorate(axes, conditional.target)
    fig.tight_layout()
    fig.savefig(results / "phase3_pi_anti_windup.png", dpi=170)
    plt.close(fig)
    return no_aw_run, aw_run


def main():
    results = Path(__file__).resolve().parents[2] / "results"
    results.mkdir(exist_ok=True)
    p_nominal, pi_nominal = make_reference_tracking_plot(results)
    p_disturbed, pi_disturbed = make_disturbance_plot(results)
    no_aw, conditional = make_anti_windup_plot(results)
    pi_settle = settling_time(pi_disturbed["time"], pi_disturbed["height"],
                              3.0, start=600.0)
    print("Phase 3 — PI control")
    print("Scenario A final height (P | PI):",
          f'{p_nominal["height"][-1]:.6f} | {pi_nominal["height"][-1]:.6f} m')
    print("Scenario B analytical PI equilibrium: h=3 m, z=7 m·s, u=0.042 m³/s")
    print("Scenario B simulated PI final:",
          f'h={pi_disturbed["height"][-1]:.6f} m,',
          f'z={pi_disturbed["integral_state"][-1]:.6f} m·s,',
          f'u={pi_disturbed["flow"][-1]:.6f} m³/s')
    print("Scenario B simulated P final:", f'h={p_disturbed["height"][-1]:.6f} m')
    print("Scenario B PI settling time after disturbance:", pi_settle, "s")
    print("Scenario C peak integral state (none | conditional):",
          f'{np.max(no_aw["integral_state"]):.3f} | {np.max(conditional["integral_state"]):.3f} m·s')
    print("Plots:", results)


if __name__ == "__main__":
    main()
