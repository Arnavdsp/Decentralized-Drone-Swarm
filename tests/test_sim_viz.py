"""
Tests for sim_viz: the adapter must be a pure observer (never changes swarm
dynamics), each scenario must trigger its documented behaviour, and metrics
must be derivable and exportable without touching the control stack.
"""
import json
import os

import numpy as np
import pytest

from swarm_drone import SwarmConfig, DroneAgent, SwarmCoordinator
from swarm_drone.backends import SimBackend
from swarm_drone.comms import LoopbackLink
from swarm_drone.load_lift import LiftPhase

from sim_viz.adapter import SimulationAdapter
from sim_viz.scenarios import SCENARIOS, run_scenario
from sim_viz.metrics import compute_summary, export_json, export_csv


def _build(cfg=None, seed=0, spread=4.0):
    cfg = cfg or SwarmConfig()
    rng = np.random.default_rng(seed)
    bus, agents = [], []
    for i in range(cfg.formation.n_drones):
        angle = 2 * np.pi * i / cfg.formation.n_drones + rng.uniform(-0.3, 0.3)
        r = rng.uniform(spread * 0.7, spread)
        start = np.array([r * np.cos(angle), r * np.sin(angle), 0.0])
        agents.append(DroneAgent(i, cfg, SimBackend(cfg.drone, position=start,
                                                     seed=seed * 10 + i),
                                  link=LoopbackLink(i, cfg.comms, bus)))
    return cfg, SwarmCoordinator(cfg, agents, centre=(0.0, 0.0, 0.0))


# ------------------------------------------------------------------- adapter
def test_adapter_is_a_pure_observer():
    """Ticking through the adapter must land the swarm in exactly the same
    state as ticking the coordinator directly with the same inputs."""
    cfg, swarm_direct = _build(seed=1)
    _, swarm_via_adapter = _build(seed=1)
    swarm_direct.start_lift(cruise_target=[6.0, 0.0])
    swarm_via_adapter.start_lift(cruise_target=[6.0, 0.0])
    adapter = SimulationAdapter(swarm_via_adapter)

    dt = 1.0 / cfg.formation.control_hz
    for _ in range(400):
        swarm_direct.tick(dt)
        adapter.tick(dt)

    np.testing.assert_allclose(swarm_direct.positions(), swarm_via_adapter.positions())
    np.testing.assert_allclose(swarm_direct.velocities(), swarm_via_adapter.velocities())
    assert swarm_direct.planner.phase == swarm_via_adapter.planner.phase


def test_adapter_reports_json_serializable_frames():
    cfg, swarm = _build()
    swarm.start_lift(cruise_target=[6.0, 0.0])
    adapter = SimulationAdapter(swarm)
    dt = 1.0 / cfg.formation.control_hz
    for _ in range(50):
        frame = adapter.tick(dt)
    json.dumps(frame.to_dict())  # must not raise
    assert len(frame.drones) == cfg.formation.n_drones
    assert frame.comm_links == [(i, (i + 1) % 6) for i in range(6)]


def test_adapter_payload_position_only_moves_once_carried():
    cfg, swarm = _build()
    swarm.start_lift(cruise_target=[6.0, 0.0])
    adapter = SimulationAdapter(swarm)
    dt = 1.0 / cfg.formation.control_hz
    frame = None
    for _ in range(60):  # still arming/takeoff/forming — nothing carried yet
        frame = adapter.tick(dt)
    assert frame.load_mass_carried_kg == 0.0
    assert frame.load_position[2] == 0.0


# ----------------------------------------------------------------- scenarios
def test_scenario_normal_completes_and_lands():
    run = run_scenario(SCENARIOS["A"]())
    assert not run.rejected
    assert run.adapter.frames[-1].phase == "landing"
    assert run.adapter.frames[-1].abort_reason is None


def test_scenario_failure_aborts_without_reassignment():
    run = run_scenario(SCENARIOS["C"]())
    assert not run.rejected
    last = run.adapter.frames[-1]
    assert last.phase == "abort"
    assert "unavailable" in last.abort_reason


def test_scenario_overload_is_rejected_before_liftoff():
    run = run_scenario(SCENARIOS["D"]())
    assert run.rejected
    assert run.feasibility_report["feasible"] is False
    assert len(run.adapter.frames) == 0


def test_scenario_disturbance_recovers_formation():
    run = run_scenario(SCENARIOS["E"]())
    rmse = np.array([f.formation_rmse_m for f in run.adapter.frames])
    phase = [f.phase for f in run.adapter.frames]
    push_idx = next(i for i, f in enumerate(run.adapter.frames) if f.event)
    spike = rmse[push_idx:push_idx + 5].max()
    # Compare within the same mission phase the push happened in — a later
    # phase transition (descend, tensioning...) moves the whole formation
    # and produces its own transient that isn't part of this recovery.
    still_same_phase = [i for i in range(push_idx, min(push_idx + 100, len(phase)))
                         if phase[i] == phase[push_idx]]
    recovered = rmse[still_same_phase[-1]]
    assert recovered < spike
    assert run.adapter.frames[-1].phase == "landing"


def test_scenario_wind_completes_with_larger_transient_error():
    calm = run_scenario(SCENARIOS["A"]())
    windy = run_scenario(SCENARIOS["B"](wind_speed=0.8, wind_start_s=0.0))
    assert windy.adapter.frames[-1].phase == "landing"
    calm_rmse = np.mean([f.formation_rmse_m for f in calm.adapter.frames])
    windy_rmse = np.mean([f.formation_rmse_m for f in windy.adapter.frames])
    assert windy_rmse >= calm_rmse


def test_scenario_respects_config_overrides():
    run = run_scenario(SCENARIOS["A"](overrides={"radius_m": 3.0, "payload_mass_kg": 2.0}))
    assert run.cfg.formation.radius_m == 3.0
    assert run.cfg.lift.payload_mass_kg == 2.0


# -------------------------------------------------------------------- metrics
def test_metrics_summary_has_expected_keys_for_completed_mission():
    run = run_scenario(SCENARIOS["A"]())
    summary = compute_summary(run)
    for key in ("formation_rmse_m_mean", "min_inter_drone_separation_m",
                "payload_displacement_m", "max_tether_tension_n",
                "mission_completion_time_s", "safety_violations"):
        assert key in summary


def test_metrics_summary_for_rejected_mission():
    run = run_scenario(SCENARIOS["D"]())
    summary = compute_summary(run)
    assert summary["rejected_before_liftoff"] is True


def test_metrics_export_round_trip(tmp_path):
    run = run_scenario(SCENARIOS["A"]())
    json_path = str(tmp_path / "run.json")
    csv_path = str(tmp_path / "run.csv")
    export_json(run, json_path)
    export_csv(run, csv_path)
    assert os.path.getsize(json_path) > 0
    assert os.path.getsize(csv_path) > 0
    with open(json_path) as fh:
        data = json.load(fh)
    assert data["scenario"] == run.name
    assert len(data["frames"]) == len(run.adapter.frames)
