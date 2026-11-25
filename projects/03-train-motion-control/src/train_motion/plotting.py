"""Consistent engineering plots for persisted train experiments."""
from pathlib import Path
import matplotlib.pyplot as plt


COLORS = {"position": "#087f83", "speed": "#2479bf", "requested": "#d87821",
          "applied": "#10263f", "integral": "#7959a8", "margin": "#2e9d64"}


def _finish(fig, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_overview(run: dict, path: Path, title: str) -> None:
    time, step_time = run["time"], run["time"][:-1]
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(time, run["x_m"], color=COLORS["position"], label="Position x")
    axes[0].axhline(run["controller"].position_target, color="#d87821", ls="--", label="Station target")
    axes[0].set_ylabel("Position (m)")
    axes[1].plot(time, run["v_mps"], color=COLORS["speed"], label="Speed v")
    axes[1].axhline(run["controller"].speed_target, color="#d87821", ls="--", label="Speed target")
    axes[1].set_ylabel("Speed (m/s)")
    axes[2].plot(step_time, run["force_requested_N"] / 1000, color=COLORS["requested"], ls="--", label="Requested")
    axes[2].plot(step_time, run["force_applied_N"] / 1000, color=COLORS["applied"], label="Applied")
    axes[2].set(xlabel="Time (s)", ylabel="Actuator force (kN)")
    axes[0].set_title(title)
    for axis in axes:
        axis.grid(alpha=.25)
        axis.legend(loc="best")
    _finish(fig, path)


def plot_forces_and_safety(run: dict, path: Path, title: str) -> None:
    step_time = run["time"][:-1]
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(step_time, run["traction_applied_N"] / 1000, label="Traction")
    axes[0].plot(step_time, -run["brake_applied_N"] / 1000, label="Active brake")
    axes[0].plot(step_time, -(run["rolling_resistance_N"] + run["velocity_resistance_N"]) / 1000, label="Passive resistance")
    axes[0].plot(step_time, -run["disturbance_N"] / 1000, label="External resistance")
    axes[0].set_ylabel("Signed force (kN)")
    axes[1].plot(run["time"], run["z_integral"], color=COLORS["integral"], label=f"Integral z ({run['integral_units'] or 'unused'})")
    axes[1].set_ylabel("Integral state")
    axes[2].plot(step_time, run["remaining_distance_m"], label="Remaining distance")
    axes[2].plot(step_time, run["estimated_stopping_distance_m"], label="Estimated stopping distance")
    axes[2].fill_between(step_time, 0, 1, where=run["supervisor_override"], transform=axes[2].get_xaxis_transform(), alpha=.16, color="#d83c21", label="Brake override")
    axes[2].set(xlabel="Time (s)", ylabel="Distance (m)")
    axes[0].set_title(title)
    for axis in axes:
        axis.grid(alpha=.25)
        axis.legend(loc="best")
    _finish(fig, path)


def plot_comparison(runs: list[tuple[str, dict]], path: Path, title: str) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for label, run in runs:
        axes[0].plot(run["time"], run["x_m"], label=label)
        axes[1].plot(run["time"], run["v_mps"], label=label)
        axes[2].plot(run["time"][:-1], run["force_applied_N"] / 1000, label=label)
    axes[0].axhline(runs[0][1]["controller"].position_target, color="black", ls="--", label="Position target")
    axes[1].axhline(runs[0][1]["controller"].speed_target, color="black", ls="--", label="Speed target")
    axes[0].set(ylabel="Position (m)", title=title)
    axes[1].set_ylabel("Speed (m/s)")
    axes[2].set(xlabel="Time (s)", ylabel="Applied force (kN)")
    for axis in axes:
        axis.grid(alpha=.25)
        axis.legend(loc="best")
    _finish(fig, path)
