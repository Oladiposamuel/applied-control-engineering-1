"""Reproducible plots for open-loop, tracking, disturbance, and windup."""
from dataclasses import replace
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from .controllers import ControllerParameters
from .model import TankParameters
from .simulation import Scenario, simulate


def _save(fig, path):
    fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)


def run_experiments(results: Path | None = None):
    results = results or Path(__file__).resolve().parents[2] / "results"
    results.mkdir(exist_ok=True)
    tank = TankParameters()

    open_run = simulate(tank, ControllerParameters(mode="open_loop"), Scenario(duration=8000))
    fig, ax = plt.subplots(2,1,figsize=(10,7),sharex=True)
    ax[0].plot(open_run["time"],open_run["h1"],label="Tank 1"); ax[0].plot(open_run["time"],open_run["h2"],label="Tank 2")
    ax[0].axhline(8.333333333,label="h1 equilibrium",ls="--"); ax[0].axhline(5,label="h2 equilibrium",ls="--")
    ax[1].plot(open_run["time"][:-1],open_run["pump_actual"],label="Pump"); ax[1].plot(open_run["time"][:-1],open_run["q12_actual"],label="q12"); ax[1].plot(open_run["time"][:-1],open_run["qout_actual"],label="Outlet")
    ax[0].set(ylabel="Height (m)",title="Two-tank open-loop response"); ax[1].set(xlabel="Time (s)",ylabel="Flow (m³/s)")
    for a in ax:a.grid(alpha=.25);a.legend()
    _save(fig,results/"open_loop_two_tank.png")

    controls=[ControllerParameters(mode="open_loop",fixed_flow=.024),ControllerParameters(mode="p"),ControllerParameters(mode="pi")]
    tracking=[simulate(tank,c,Scenario(duration=8000)) for c in controls]
    fig,ax=plt.subplots(3,1,figsize=(10,9),sharex=True)
    for run,label in zip(tracking,["Open loop bias","P","PI"]):
        ax[0].plot(run["time"],run["h2"],label=label);ax[1].plot(run["time"],run["h1"],label=label);ax[2].plot(run["time"][:-1],run["pump_actual"],label=label)
    ax[0].axhline(3,color="black",ls="--",label="Target");ax[0].set(ylabel="Tank 2 (m)",title="Tank 2 reference tracking");ax[1].set(ylabel="Tank 1 (m)");ax[2].set(xlabel="Time (s)",ylabel="Pump (m³/s)")
    for a in ax:a.grid(alpha=.25);a.legend()
    _save(fig,results/"tracking_comparison.png")

    equilibrium_tank=replace(tank,initial_height1=5,initial_height2=3)
    scenario=Scenario(duration=12000,disturbance_at=600,demand=.006)
    disturbed=[simulate(equilibrium_tank,c,scenario) for c in controls]
    fig,ax=plt.subplots(3,1,figsize=(10,9),sharex=True)
    for run,label in zip(disturbed,["Open loop bias","P","PI"]):
        ax[0].plot(run["time"],run["h2"],label=label);ax[1].plot(run["time"],run["h1"],label=label);ax[2].plot(run["time"][:-1],run["pump_actual"],label=label)
    for a in ax:a.axvline(600,color="black",ls=":",label="Demand starts");a.grid(alpha=.25);a.legend()
    ax[0].axhline(3,color="black",ls="--");ax[0].set(ylabel="Tank 2 (m)",title="Persistent Tank 2 demand rejection");ax[1].set(ylabel="Tank 1 (m)");ax[2].set(xlabel="Time (s)",ylabel="Pump (m³/s)")
    _save(fig,results/"disturbance_rejection.png")

    stress_tank=replace(tank,initial_height1=1,initial_height2=.2)
    base=ControllerParameters(mode="pi",target=6,kp=.03,ki=.0004,max_flow=.06)
    windup=[simulate(stress_tank,replace(base,anti_windup=a),Scenario(duration=5000)) for a in ("none","conditional")]
    fig,ax=plt.subplots(3,1,figsize=(10,9),sharex=True)
    for run,label in zip(windup,["No anti-windup","Conditional"]):
        ax[0].plot(run["time"],run["h2"],label=label);ax[1].plot(run["time"],run["integral_state"],label=label);ax[2].plot(run["time"][:-1],run["pump_raw"],ls="--",label=f"Raw — {label}");ax[2].plot(run["time"][:-1],run["pump_actual"],label=f"Actual — {label}")
    ax[0].axhline(6,color="black",ls="--");ax[0].set(ylabel="Tank 2 (m)",title="PI anti-windup stress case");ax[1].set(ylabel="z (m·s)");ax[2].set(xlabel="Time (s)",ylabel="Pump (m³/s)")
    for a in ax:a.grid(alpha=.25);a.legend(fontsize=8)
    _save(fig,results/"anti_windup.png")

    pi=disturbed[2]
    print(f"Open-loop final: h1={open_run['h1'][-1]:.6f}, h2={open_run['h2'][-1]:.6f}")
    print(f"Disturbed P final h2={disturbed[1]['h2'][-1]:.6f}")
    print(f"Disturbed PI final: h1={pi['h1'][-1]:.6f}, h2={pi['h2'][-1]:.6f}, z={pi['integral_state'][-1]:.6f}, u={pi['pump_actual'][-1]:.6f}")
    print(f"Maximum mass residual={max(np.max(np.abs(r['volume_balance_residual'])) for r in disturbed):.3e} m³")
    return {"open":open_run,"tracking":tracking,"disturbed":disturbed,"windup":windup}


if __name__ == "__main__":
    run_experiments()
