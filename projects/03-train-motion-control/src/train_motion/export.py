"""CSV and JSON export helpers for deterministic experiment records."""
from dataclasses import asdict
import csv
import json
from pathlib import Path
import platform
import sys
import numpy as np


CSV_FIELDS = (
    "t", "x_m", "v_mps", "a_mps2", "x_ref_m", "v_ref_mps", "e_x_m",
    "e_v_mps", "z_integral", "force_requested_N", "force_applied_N",
    "traction_applied_N", "brake_applied_N", "rolling_resistance_N",
    "velocity_resistance_N", "disturbance_N", "net_force_N",
    "is_saturated", "anti_windup_active", "remaining_distance_m",
    "estimated_stopping_distance_m", "stopping_margin_m", "feasible_stop",
    "supervisor_override", "stopped", "stop_event",
)


def _json_dump(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def export_run(directory: Path, run: dict, metrics: dict,
               experiment_name: str, notes: str = "") -> None:
    directory.mkdir(parents=True, exist_ok=True)
    config = {
        "experiment": experiment_name,
        "notes": notes,
        "plant": asdict(run["plant"]),
        "controller": asdict(run["controller"]),
        "scenario": asdict(run["scenario"]),
        "integral_state_units": run["integral_units"],
        "software": {"python": platform.python_version(), "numpy": np.__version__, "implementation": sys.implementation.name},
    }
    _json_dump(directory / "config.json", config)
    _json_dump(directory / "metrics.json", metrics)
    state_keys = {"x_m", "v_mps", "z_integral"}
    with (directory / "timeseries.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, lineterminator="\n")
        writer.writeheader()
        steps = len(run["time"]) - 1
        for index, timestamp in enumerate(run["time"]):
            row = {"t": f"{timestamp:.10g}"}
            for field in CSV_FIELDS[1:]:
                mode = run["controller"].mode
                if ((field in {"x_ref_m", "e_x_m"} and not mode.startswith("position_")) or
                    (field in {"v_ref_mps", "e_v_mps"} and not mode.startswith("speed_")) or
                    (field == "z_integral" and run["integral_units"] is None)):
                    row[field] = ""
                    continue
                if field in state_keys:
                    value = run[field][index]
                elif index < steps:
                    value = run[field][index]
                else:
                    row[field] = ""
                    continue
                if isinstance(value, (bool, np.bool_)):
                    row[field] = "1" if value else "0"
                else:
                    row[field] = f"{float(value):.12g}"
            writer.writerow(row)
