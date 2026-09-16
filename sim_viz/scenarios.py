"""
The five demo scenarios, built entirely from the existing ``swarm_drone``
API plus the observing ``SimulationAdapter``. No scenario here changes a
control gain, a physics constant, or the mission state machine — each one
only decides *what to inject* (wind, a failed drone, a shove, an oversized
payload) and *when*.

Every builder returns a ``ScenarioRun``: a ready-to-arm ``SwarmCoordinator``
wrapped in a ``SimulationAdapter``, plus a list of ``(time_s, fn)`` scripted
events the driving loop (``server.py`` or ``run_visual_sim.py``) applies as
it steps through time.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import numpy as np

from swarm_drone import SwarmConfig, DroneAgent, SwarmCoordinator
from swarm_drone.backends import SimBackend
from swarm_drone.comms import LoopbackLink
from swarm_drone.load_lift import feasibility

from .adapter import SimulationAdapter

Event = Tuple[float, str, Callable[[SimulationAdapter], None]]


@dataclass
class ScenarioRun:
    name: str
    description: str
    cfg: SwarmConfig
    adapter: SimulationAdapter
    events: List[Event] = field(default_factory=list)
    duration_s: float = 90.0
    feasibility_report: Optional[dict] = None
    rejected: bool = False  # True if the mission never leaves the ground (Scenario D)


def build_swarm(cfg: SwarmConfig, seed: int = 0, spread: float = 4.0) -> SwarmCoordinator:
    """Six drones scattered on the ground around the pickup point — identical
    to ``scripts/run_sim.py::build_swarm``, reused so both entry points start
    from the same distribution."""
    rng = np.random.default_rng(seed)
    bus, agents = [], []
    for i in range(cfg.formation.n_drones):
        angle = 2 * np.pi * i / cfg.formation.n_drones + rng.uniform(-0.35, 0.35)
        radius = rng.uniform(spread * 0.7, spread)
        start = np.array([radius * np.cos(angle), radius * np.sin(angle), 0.0])
        backend = SimBackend(cfg.drone, position=start, seed=seed * 10 + i)
        agents.append(DroneAgent(i, cfg, backend,
                                  link=LoopbackLink(i, cfg.comms, bus)))
    return SwarmCoordinator(cfg, agents, centre=(0.0, 0.0, 0.0))


def _base_cfg(overrides: Optional[dict] = None) -> SwarmConfig:
    cfg = SwarmConfig()
    overrides = overrides or {}
    if "n_drones" in overrides:
        cfg.formation.n_drones = int(overrides["n_drones"])
    if "radius_m" in overrides:
        cfg.formation.radius_m = float(overrides["radius_m"])
    if "payload_mass_kg" in overrides:
        cfg.lift.payload_mass_kg = float(overrides["payload_mass_kg"])
    if "tether_len_m" in overrides:
        cfg.formation.tether_len_m = float(overrides["tether_len_m"])
    if "tether_stiffness_n_per_m" in overrides:
        cfg.lift.tether_stiffness_n_per_m = float(overrides["tether_stiffness_n_per_m"])
    if "k_formation" in overrides:
        cfg.formation.k_formation = float(overrides["k_formation"])
    if "k_consensus" in overrides:
        cfg.formation.k_consensus = float(overrides["k_consensus"])
    if "k_integral" in overrides:
        cfg.formation.k_integral = float(overrides["k_integral"])
    if "k_damping" in overrides:
        cfg.formation.k_damping = float(overrides["k_damping"])
    if "control_hz" in overrides:
        cfg.formation.control_hz = float(overrides["control_hz"])
    return cfg


# ------------------------------------------------------------- A: normal mission
def scenario_normal(overrides: Optional[dict] = None, seed: int = 0) -> ScenarioRun:
    cfg = _base_cfg(overrides)
    swarm = build_swarm(cfg, seed=seed)
    adapter = SimulationAdapter(swarm)
    report = swarm.start_lift(cruise_target=[8.0, 0.0])
    return ScenarioRun(
        name="A — Normal mission",
        description="Arm, take off, form the hexagon, descend, tension the "
                     "tethers, lift, cruise, lower, release, land.",
        cfg=cfg, adapter=adapter, events=[], duration_s=90.0,
        feasibility_report=report, rejected=not report["feasible"])


# ------------------------------------------------------------- B: wind
def scenario_wind(overrides: Optional[dict] = None, seed: int = 0,
                   wind_speed: float = 0.8, wind_dir_deg: float = 0.0,
                   wind_start_s: float = 0.0) -> ScenarioRun:
    cfg = _base_cfg(overrides)
    swarm = build_swarm(cfg, seed=seed)
    adapter = SimulationAdapter(swarm)
    report = swarm.start_lift(cruise_target=[8.0, 0.0])

    theta = np.radians(wind_dir_deg)
    vector = np.array([wind_speed * np.cos(theta), wind_speed * np.sin(theta), 0.0])

    def apply_wind(ad: SimulationAdapter):
        ad.set_wind(vector)

    events = [(wind_start_s, "wind_on", apply_wind)]
    return ScenarioRun(
        name="B — Wind",
        description=f"Steady {wind_speed:.1f} m/s^2 wind from t={wind_start_s:.0f}s. "
                     "The integral and damping terms should pull the formation "
                     "error back down after the initial hit.",
        cfg=cfg, adapter=adapter, events=events, duration_s=90.0,
        feasibility_report=report, rejected=not report["feasible"])


# ------------------------------------------------------------- C: drone failure
def scenario_failure(overrides: Optional[dict] = None, seed: int = 0,
                      fail_drone: int = 3, fail_at_s: float = 20.0) -> ScenarioRun:
    cfg = _base_cfg(overrides)
    swarm = build_swarm(cfg, seed=seed)
    adapter = SimulationAdapter(swarm)
    report = swarm.start_lift(cruise_target=[8.0, 0.0])

    def cause_failure(ad: SimulationAdapter):
        ad.fail_drone(fail_drone, reason="simulated motor failure")

    events = [(fail_at_s, f"drone_{fail_drone}_failed", cause_failure)]
    return ScenarioRun(
        name="C — Drone failure",
        description=f"Drone {fail_drone} fails at t={fail_at_s:.0f}s. The "
                     "coordinator does not reassign the payload — it aborts "
                     "the lift and lands, exactly as swarm.py already does.",
        cfg=cfg, adapter=adapter, events=events, duration_s=90.0,
        feasibility_report=report, rejected=not report["feasible"])


# ------------------------------------------------------------- D: overload
def scenario_overload(overrides: Optional[dict] = None, seed: int = 0,
                       payload_mass_kg: float = 20.0) -> ScenarioRun:
    overrides = dict(overrides or {})
    overrides["payload_mass_kg"] = payload_mass_kg
    cfg = _base_cfg(overrides)
    swarm = build_swarm(cfg, seed=seed)
    adapter = SimulationAdapter(swarm)
    report = swarm.start_lift(cruise_target=[8.0, 0.0])
    return ScenarioRun(
        name="D — Overload",
        description=f"{payload_mass_kg:.1f} kg payload exceeds the swarm's "
                     "safety-margined capacity. ``LiftPlanner.start`` refuses "
                     "the mission before arming — nothing ever leaves the ground.",
        cfg=cfg, adapter=adapter, events=[], duration_s=15.0,
        feasibility_report=report, rejected=not report["feasible"])


# ------------------------------------------------------------- E: disturbance
def scenario_disturbance(overrides: Optional[dict] = None, seed: int = 0,
                          push_drone: int = 0, push_at_s: float = 5.0,
                          push_offset=(1.5, 0.0, 0.0)) -> ScenarioRun:
    cfg = _base_cfg(overrides)
    swarm = build_swarm(cfg, seed=seed)
    adapter = SimulationAdapter(swarm)
    report = swarm.start_lift(cruise_target=[8.0, 0.0])

    def push(ad: SimulationAdapter):
        ad.push_drone(push_drone, push_offset)

    events = [(push_at_s, f"drone_{push_drone}_pushed", push)]
    return ScenarioRun(
        name="E — Formation disturbance",
        description=f"Drone {push_drone} is shoved {np.linalg.norm(push_offset):.1f} m "
                     f"off its slot at t={push_at_s:.0f}s. Formation + consensus + "
                     "avoidance terms should pull it back in. Default timing is "
                     "before the tethers tension, so this shows pure formation "
                     "recovery; push it during LIFTING/CRUISE instead (larger "
                     "push_at_s) to see the load-imbalance safety abort trip.",
        cfg=cfg, adapter=adapter, events=events, duration_s=90.0,
        feasibility_report=report, rejected=not report["feasible"])


SCENARIOS = {
    "A": scenario_normal,
    "B": scenario_wind,
    "C": scenario_failure,
    "D": scenario_overload,
    "E": scenario_disturbance,
}


def run_scenario(run: ScenarioRun, dt: Optional[float] = None,
                  on_frame: Optional[Callable] = None) -> ScenarioRun:
    """Drive a ``ScenarioRun`` to completion synchronously (headless), firing
    scripted events at their timestamps and calling ``on_frame`` (if given)
    once per tick — used by both the validation script and the live server's
    per-tick callback."""
    if run.rejected:
        return run  # Scenario D: nothing to tick, the mission never armed.

    dt = dt or (1.0 / run.cfg.formation.control_hz)
    pending = sorted(run.events, key=lambda e: e[0])
    fired = set()
    steps = int(run.duration_s / dt)

    for step in range(steps):
        t = step * dt
        event_name = None
        for idx, (when, name, fn) in enumerate(pending):
            if idx not in fired and t >= when:
                fn(run.adapter)
                fired.add(idx)
                event_name = name

        frame = run.adapter.tick(dt, event=event_name)
        if on_frame:
            on_frame(frame)

        if frame.phase in ("landing", "abort") and \
                max(d["position"][2] for d in frame.drones) < 0.05 and frame.t > 0.5:
            break

    return run
