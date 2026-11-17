"""Project 01: single-tank open-loop dynamics and numerical validation.

Run from the workspace root:
    PYTHONPATH=projects/01-water-tank/src python -m water_tank.tank_simulation
"""
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


@dataclass(frozen=True)
class TankParameters:
    area: float = 2.0                 # m^2
    inlet_flow: float = 0.04          # m^3/s
    outlet_coefficient: float = 0.01  # m^2/s: outlet flow = coefficient * height
    leak_flow: float = 0.005          # m^3/s
    initial_height: float = 1.0       # m

    def __post_init__(self):
        if self.area <= 0 or self.outlet_coefficient <= 0:
            raise ValueError("Area and outlet coefficient must be positive")
        if self.initial_height < 0 or self.inlet_flow < 0 or self.leak_flow < 0:
            raise ValueError("Initial height and flow rates cannot be negative")


def height_rate(height: float, p: TankParameters) -> float:
    """dh/dt = (inflow - height-dependent outflow - leak) / area."""
    return (p.inlet_flow - p.outlet_coefficient * height - p.leak_flow) / p.area


def equilibrium_height(p: TankParameters) -> float:
    return (p.inlet_flow - p.leak_flow) / p.outlet_coefficient


def analytical_height(times: np.ndarray, p: TankParameters) -> np.ndarray:
    """Exact solution for constant inlet and leak in the linear tank model."""
    h_eq = equilibrium_height(p)
    tau = p.area / p.outlet_coefficient
    return h_eq + (p.initial_height - h_eq) * np.exp(-times / tau)


def forward_euler(p: TankParameters, duration: float, dt: float):
    if dt <= 0 or duration <= 0:
        raise ValueError("Duration and time step must be positive")
    steps_exact = duration / dt
    steps = round(steps_exact)
    if not np.isclose(steps_exact, steps, rtol=0, atol=1e-9):
        raise ValueError("Choose a dt that divides duration exactly")
    times = np.linspace(0, duration, steps + 1)
    heights = np.empty(steps + 1, dtype=float)
    heights[0] = p.initial_height
    for k in range(steps):
        heights[k + 1] = heights[k] + dt * height_rate(heights[k], p)
    return times, heights


def main():
    p = TankParameters()
    duration = 1000.0
    dt = 0.1
    times, euler = forward_euler(p, duration, dt)
    exact = analytical_height(times, p)
    errors = np.abs(euler - exact)
    tau = p.area / p.outlet_coefficient

    print(f"Equilibrium height : {equilibrium_height(p):.6f} m")
    print(f"Initial rate       : {height_rate(p.initial_height, p):.6f} m/s")
    print(f"Time constant      : {tau:.1f} s")
    print(f"Euler h(0.1 s)     : {euler[1]:.9f} m")
    print(f"Euler h(0.2 s)     : {euler[2]:.9f} m")
    print(f"Max absolute error : {errors.max():.9f} m")
    print("\nTime-step comparison (absolute error at t = 200 seconds):")
    for trial_dt in (0.1, 1.0, 10.0, 100.0, 500.0):
        t_trial, h_trial = forward_euler(p, 2000.0, trial_dt)
        target_step = int(round(200.0 / trial_dt)) if 200.0 / trial_dt >= 1 else None
        if target_step is not None and np.isclose(t_trial[target_step], 200.0):
            error = abs(h_trial[target_step] - analytical_height(np.array([200.0]), p)[0])
            print(f"dt = {trial_dt:6.1f} s -> error = {error:.6f} m")
        else:
            error = abs(h_trial[-1] - analytical_height(np.array([2000.0]), p)[0])
            print(f"dt = {trial_dt:6.1f} s -> error at 2000 s = {error:.6f} m")

    results = Path(__file__).resolve().parents[2] / "results"
    results.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(times, exact, label="Analytical solution", linewidth=2)
    ax.plot(times, euler, "--", label=f"Forward Euler (dt={dt} s)", linewidth=1.5)
    ax.axhline(equilibrium_height(p), linestyle=":", label="Equilibrium")
    ax.set(xlabel="Time (s)", ylabel="Water height (m)", title="Tank: Forward Euler vs exact solution")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(results / "tank_response.png", dpi=170)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 5))
    for trial_dt in (0.1, 10.0, 100.0, 500.0):
        t_trial, h_trial = forward_euler(p, duration=2000.0, dt=trial_dt)
        ax.plot(t_trial, h_trial, label=f"Euler dt={trial_dt:g} s")
    t_exact = np.linspace(0, 2000, 1001)
    ax.plot(t_exact, analytical_height(t_exact, p), "k--", label="Exact", linewidth=1.5)
    ax.set(xlabel="Time (s)", ylabel="Water height (m)", title="Effect of simulation time step")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(results / "timestep_comparison.png", dpi=170)
    plt.close(fig)
    print(f"\nSaved plots to {results}")


if __name__ == "__main__":
    main()
