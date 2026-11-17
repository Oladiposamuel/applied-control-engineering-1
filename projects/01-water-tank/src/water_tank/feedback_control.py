"""Phase 2: sampled proportional feedback for a single cylindrical tank.

Run from the workspace root:
PYTHONPATH=projects/01-water-tank/src python -m water_tank.feedback_control
The controller is evaluated once every dt seconds and its output is held
constant throughout that interval. Forward Euler advances the tank model.
"""
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from .tank_simulation import TankParameters


@dataclass(frozen=True)
class ControllerParameters:
    kp: float = 0.03                  # m^2/s
    target: float = 3.0               # m
    bias: float = 0.035               # m^3/s
    min_flow: float = 0.0             # m^3/s
    max_flow: float = 0.10            # m^3/s

    def __post_init__(self):
        if self.kp < 0 or self.target < 0 or self.bias < 0:
            raise ValueError('Gain, target and bias must be nonnegative')
        if self.min_flow < 0 or self.max_flow <= self.min_flow:
            raise ValueError('Invalid pump limits')


def inlet_command(height: float, control: ControllerParameters) -> float:
    """Saturated P law; height in m, flow out in m^3/s."""
    raw = control.bias + control.kp * (control.target - height)
    return float(np.clip(raw, control.min_flow, control.max_flow))


def simulate(p: TankParameters, control: ControllerParameters | None,
             *, duration: float = 1200.0, dt: float = 0.1,
             disturbance_at: float | None = None, extra_leak: float = 0.0):
    """Forward Euler with zero-order-held pump command.

    control=None means constant open-loop inlet flow p.inlet_flow.
    An optional step disturbance adds extra leak from disturbance_at onward.
    """
    if duration <= 0 or dt <= 0 or extra_leak < 0:
        raise ValueError('Duration and dt must be positive; extra leak nonnegative')
    if disturbance_at is not None and not (0 <= disturbance_at <= duration):
        raise ValueError('Disturbance time outside simulation interval')
    n_float = duration / dt
    n = round(n_float)
    if not np.isclose(n_float, n, atol=1e-8, rtol=0):
        raise ValueError('dt must divide duration')
    time = np.linspace(0, duration, n + 1)
    height = np.zeros(n + 1)
    height[0] = p.initial_height
    flow = np.empty(n)
    leak = np.empty(n)
    for i in range(n):
        flow[i] = p.inlet_flow if control is None else inlet_command(height[i], control)
        leak[i] = p.leak_flow + (extra_leak if disturbance_at is not None and time[i] >= disturbance_at - 1e-9 else 0.0)
        rate = (flow[i] - p.outlet_coefficient * height[i] - leak[i]) / p.area
        height[i+1] = height[i] + dt * rate
        # This model intentionally does not clamp height: parameters are chosen
        # to keep height nonnegative; nonphysical scenarios need boundary model.
    return {'time': time, 'height': height, 'flow': flow, 'leak': leak}


def steady_state_height(p: TankParameters, control: ControllerParameters,
                        *, extra_leak: float = 0.0) -> float:
    """Exact closed-loop equilibrium when pump saturation is inactive."""
    k = p.outlet_coefficient
    return (control.bias + control.kp * control.target - p.leak_flow - extra_leak) / (k + control.kp)


def settling_time(time, height, target, tolerance=0.02, start=0.0):
    """First time after start at which all later samples stay within tolerance (m).

    Returns None when the run does not settle within the observation period.
    """
    idx0 = np.searchsorted(time, start, side='left')
    deviations = np.abs(height - target)
    failures = np.flatnonzero(deviations[idx0:] > tolerance)
    if len(failures) == 0:
        return float(time[idx0] - start)
    last = idx0 + failures[-1]
    if last >= len(time) - 1:
        return None
    return float(time[last + 1] - start)


def make_plots(runs: dict, results: Path, target: float):
    results.mkdir(exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True,
                             gridspec_kw={'height_ratios': [2, 1]})
    for label, run in runs.items():
        axes[0].plot(run['time'], run['height'], label=label, linewidth=1.7)
        axes[1].step(run['time'][:-1], run['flow'], where='post', label=label, linewidth=1.4)
    axes[0].axhline(target, color='black', linestyle='--', label='Target: 3 m', linewidth=1)
    axes[0].set(ylabel='Water height (m)', title='Open-loop vs P control: no disturbance')
    axes[1].set(xlabel='Time (s)', ylabel='Inlet flow (m³/s)')
    for ax in axes:
        ax.grid(alpha=.25)
        ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(results / 'phase2_controller_comparison.png', dpi=160)
    plt.close(fig)



def main():
    p = TankParameters()
    biased = ControllerParameters()
    unbiased = ControllerParameters(bias=0.0)
    runs = {
        'Open loop': simulate(p, None),
        'P without bias': simulate(p, unbiased),
        'P with equilibrium bias': simulate(p, biased),
    }
    disturbance_at = 600.0
    upset = {
        'Open loop': simulate(p, None, disturbance_at=disturbance_at, extra_leak=0.01),
        'P without bias': simulate(p, unbiased, disturbance_at=disturbance_at, extra_leak=0.01),
        'P with equilibrium bias': simulate(p, biased, disturbance_at=disturbance_at, extra_leak=0.01),
    }
    results = Path(__file__).resolve().parents[2] / 'results'
    make_plots(runs, results, biased.target)
    # separate chart for disturbance (same plot mechanics, different title)
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True,
                             gridspec_kw={'height_ratios': [2, 1]})
    for label, run in upset.items():
        axes[0].plot(run['time'], run['height'], label=label, linewidth=1.7)
        axes[1].step(run['time'][:-1], run['flow'], where='post', label=label, linewidth=1.4)
    axes[0].axhline(biased.target, color='black', linestyle='--', linewidth=1, label='Target: 3 m')
    for ax in axes:
        ax.axvline(disturbance_at, linestyle=':', color='black', alpha=.6, label='Extra leak starts')
        ax.grid(alpha=.25)
        ax.legend(fontsize=9)
    axes[0].set(ylabel='Height (m)', title='Disturbance rejection: extra leakage of 0.01 m³/s at t=600 s')
    axes[1].set(xlabel='Time (s)', ylabel='Inlet flow (m³/s)')
    fig.tight_layout()
    fig.savefig(results / 'phase2_disturbance_response.png', dpi=160)
    plt.close(fig)

    print('Initial P-with-bias pump command:', inlet_command(p.initial_height, biased), 'm³/s')
    print('Predicted undisturbed equilibria:')
    print('  P without bias:', steady_state_height(p, unbiased), 'm')
    print('  P with bias:   ', steady_state_height(p, biased), 'm')
    print('Predicted equilibria with extra leakage:')
    print('  P without bias:', steady_state_height(p, unbiased, extra_leak=0.01), 'm')
    print('  P with bias:   ', steady_state_height(p, biased, extra_leak=0.01), 'm')
    print('\nEnd-of-run height (no disturbance | disturbance):')
    for name in runs:
        print(f'  {name:25s}: {runs[name]["height"][-1]:.6f} | {upset[name]["height"][-1]:.6f} m')
    print('\nSettling to within ±0.02 m of reference, undisturbed:')
    for name in runs:
        print(f'  {name:25s}: {settling_time(runs[name]["time"], runs[name]["height"], biased.target)} s')
    print('\nPlots:', results)


if __name__ == '__main__':
    main()
