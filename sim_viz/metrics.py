"""
Post-hoc metrics over a completed ``ScenarioRun`` — every number here is
derived from ``FrameSnapshot`` fields the adapter already captured, so this
module never touches the control stack or re-simulates anything.
"""
from __future__ import annotations

import csv
import json
import os
from typing import Optional

import numpy as np


def compute_summary(run) -> dict:
    """Phase-7 metrics table for one ``ScenarioRun``."""
    frames = run.adapter.frames

    if run.rejected or not frames:
        return {
            "scenario": run.name,
            "rejected_before_liftoff": True,
            "feasibility": run.feasibility_report,
        }

    rmse = np.array([f.formation_rmse_m for f in frames])
    min_sep = np.array([f.min_separation_m for f in frames])
    load_pos = np.array([f.load_position for f in frames])
    load_mass = np.array([f.load_mass_carried_kg for f in frames])
    shares = np.array([[d["load_share_n"] for d in f.drones] for f in frames])
    tensions = np.array([[d["tether_tension_n"] for d in f.drones] for f in frames])
    batteries = np.array([[d["battery_frac"] for d in f.drones] for f in frames])
    t = np.array([f.t for f in frames])

    loaded_mask = load_mass > 1e-6
    if loaded_mask.any():
        fair = shares[loaded_mask].sum(axis=1, keepdims=True) / shares.shape[1]
        with np.errstate(invalid="ignore", divide="ignore"):
            ratio = np.where(fair > 1e-9, shares[loaded_mask] / np.maximum(fair, 1e-9), np.nan)
        max_share_ratio = float(np.nanmax(ratio))
        avg_load_share_n = float(np.nanmean(shares[loaded_mask]))
        max_tension_n = float(np.nanmax(tensions[loaded_mask]))
    else:
        max_share_ratio = 0.0
        avg_load_share_n = 0.0
        max_tension_n = 0.0

    start_xy = load_pos[loaded_mask][0, :2] if loaded_mask.any() else load_pos[0, :2]
    payload_displacement_m = float(np.max(
        np.linalg.norm(load_pos[:, :2] - start_xy, axis=1))) if len(load_pos) else 0.0

    battery_drop = float((batteries[0] - batteries[-1]).mean())

    return {
        "scenario": run.name,
        "rejected_before_liftoff": False,
        "final_phase": frames[-1].phase,
        "abort_reason": frames[-1].abort_reason,
        "mission_completion_time_s": float(t[-1]),
        "formation_rmse_m_mean": float(rmse.mean()),
        "formation_rmse_m_max": float(rmse.max()),
        "min_inter_drone_separation_m": float(min_sep.min()),
        "payload_displacement_m": payload_displacement_m,
        "payload_altitude_m_max": float(load_pos[:, 2].max()) if len(load_pos) else 0.0,
        "average_load_share_n": avg_load_share_n,
        "max_load_share_ratio": max_share_ratio,
        "max_tether_tension_n": max_tension_n,
        "safety_violations": int(run.adapter.safety_violations),
        "battery_consumed_frac_mean": battery_drop,
    }


def export_json(run, path: str) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    payload = {
        "scenario": run.name,
        "description": run.description,
        "config": run.cfg.to_dict(),
        "feasibility": run.feasibility_report,
        "summary": compute_summary(run),
        "frames": [f.to_dict() for f in run.adapter.frames],
    }
    with open(path, "w") as fh:
        json.dump(payload, fh, indent=1)


def export_csv(run, path: str) -> None:
    """One row per tick per drone — the flattest useful shape for a CSV."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fieldnames = [
        "t", "phase", "shape_quality", "formation_rmse_m", "min_separation_m",
        "load_mass_carried_kg", "load_x", "load_y", "load_z",
        "drone_id", "healthy", "battery_frac", "pos_x", "pos_y", "pos_z",
        "formation_error_m", "load_share_n", "tether_tension_n",
    ]
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for f in run.adapter.frames:
            for d in f.drones:
                writer.writerow({
                    "t": f.t, "phase": f.phase, "shape_quality": f.shape_quality,
                    "formation_rmse_m": f.formation_rmse_m,
                    "min_separation_m": f.min_separation_m,
                    "load_mass_carried_kg": f.load_mass_carried_kg,
                    "load_x": f.load_position[0], "load_y": f.load_position[1],
                    "load_z": f.load_position[2],
                    "drone_id": d["id"], "healthy": d["healthy"],
                    "battery_frac": d["battery_frac"],
                    "pos_x": d["position"][0], "pos_y": d["position"][1],
                    "pos_z": d["position"][2],
                    "formation_error_m": d["formation_error_m"],
                    "load_share_n": d["load_share_n"],
                    "tether_tension_n": d["tether_tension_n"],
                })
