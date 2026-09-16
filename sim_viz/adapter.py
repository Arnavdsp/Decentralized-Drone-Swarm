"""
Simulation adapter: observes an existing ``SwarmCoordinator`` after each tick
and produces a rich, render-ready snapshot — without adding a single line of
rendering logic to the control stack itself.

    existing SwarmCoordinator.tick()
                |
                v
    SimulationAdapter.tick()   <-- this file only *reads* public state
                |
                v
    FrameSnapshot (plain dataclass, JSON-serializable)
                |
                v
    web renderer / metrics recorder / CSV-JSON export

Every field below already exists somewhere on the coordinator, its agents, or
its planner (see ``docs/SIMULATOR.md`` for the exact attribute each one comes
from). The one thing that genuinely does not exist upstream is the payload's
*world position* as a per-tick series — ``LiftPlanner.current_shares`` solves
for ``load_z`` on demand but nothing stores it — so the adapter recomputes it
each frame from the same public ``solve_load_equilibrium`` call the planner
itself uses. That is a read, not a change to the physics.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict, field
from typing import List, Optional, Tuple

import numpy as np

from swarm_drone.geometry import hexagon_slots
from swarm_drone.load_lift import solve_load_equilibrium

G = 9.80665


@dataclass
class DroneSnapshot:
    """Everything about one drone at one instant — enough for the 3D view
    and the debug HUD without re-deriving anything from raw arrays."""
    id: int
    slot: int
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    target: Tuple[float, float, float]
    accel_cmd: Tuple[float, float, float]
    formation_error_m: float
    healthy: bool
    fault: Optional[str]
    battery_frac: float
    load_share_n: float
    tether_tension_n: float

    def to_dict(self):
        return asdict(self)


@dataclass
class FrameSnapshot:
    """One control tick of the whole swarm, ready to hand to a renderer."""
    t: float
    phase: str
    alt_setpoint: float
    shape_quality: float
    min_separation_m: float
    formation_rmse_m: float
    load_position: Tuple[float, float, float]
    load_mass_carried_kg: float
    wind: Tuple[float, float, float]
    comm_links: List[Tuple[int, int]]
    drones: List[dict]
    abort_reason: Optional[str]
    event: Optional[str] = None  # scenario-scripted marker, e.g. "drone_3_failed"

    def to_dict(self):
        return {
            "t": self.t,
            "phase": self.phase,
            "alt_setpoint": self.alt_setpoint,
            "shape_quality": self.shape_quality,
            "min_separation_m": self.min_separation_m,
            "formation_rmse_m": self.formation_rmse_m,
            "load_position": self.load_position,
            "load_mass_carried_kg": self.load_mass_carried_kg,
            "wind": self.wind,
            "comm_links": self.comm_links,
            "drones": self.drones,
            "abort_reason": self.abort_reason,
            "event": self.event,
        }


def _ring_links(n: int) -> List[Tuple[int, int]]:
    """Undirected ring edges (i, i+1 mod n) — matches geometry.ring_neighbours."""
    return [(i, (i + 1) % n) for i in range(n)]


class SimulationAdapter:
    """Wraps one ``SwarmCoordinator`` and turns each ``tick()`` into a
    ``FrameSnapshot``. Holds no control state of its own — every number it
    reports is read from the coordinator, its agents, or its planner after
    they have already updated themselves.
    """

    def __init__(self, swarm):
        self.swarm = swarm
        self.cfg = swarm.cfg
        self.comm_links = _ring_links(len(swarm.agents))
        self.frames: List[FrameSnapshot] = []
        self.safety_violations = 0
        self._last_abort_reason = None

    # ------------------------------------------------------------- actuation
    def set_wind(self, vector) -> None:
        """Apply a wind vector to every drone's backend (Scenario B)."""
        vector = np.asarray(vector, dtype=float)
        for agent in self.swarm.agents:
            if hasattr(agent.backend, "set_wind"):
                agent.backend.set_wind(vector)

    def fail_drone(self, drone_id: int, reason: str = "simulated motor failure") -> None:
        """Mark a drone unhealthy (Scenario C). The coordinator's own
        ``tick()`` already aborts a loaded mission when it sees this."""
        self.swarm.agents[drone_id].fail(reason)

    def push_drone(self, drone_id: int, offset) -> None:
        """Teleport one drone off its slot (Scenario E) by nudging the
        backend's position directly — the formation controller's avoidance
        and consensus terms do the rest on the next tick."""
        offset = np.asarray(offset, dtype=float)
        backend = self.swarm.agents[drone_id].backend
        backend.pos = backend.pos + offset

    # ------------------------------------------------------------- stepping
    def tick(self, dt: float, event: Optional[str] = None) -> FrameSnapshot:
        phase = self.swarm.tick(dt)

        positions = self.swarm.positions()
        velocities = self.swarm.velocities()
        formation = self.swarm.formation
        planner = self.swarm.planner

        errors = formation.errors(positions)
        shares = self.swarm.load_shares()  # also refreshes planner.load_z / .tensions

        wind = np.zeros(3)
        if self.swarm.agents and hasattr(self.swarm.agents[0].backend, "wind"):
            wind = np.asarray(self.swarm.agents[0].backend.wind, dtype=float)

        load_mass = planner.load_mass_carried()
        if load_mass > 1e-9:
            centre_xy = positions[:, :2].mean(axis=0)
            load_pos = (float(centre_xy[0]), float(centre_xy[1]), float(planner.load_z))
        else:
            centre_xy = positions[:, :2].mean(axis=0)
            load_pos = (float(centre_xy[0]), float(centre_xy[1]), 0.0)

        min_sep = self._min_separation(positions)

        drones = []
        for i, agent in enumerate(self.swarm.agents):
            drones.append(DroneSnapshot(
                id=agent.id,
                slot=agent.slot,
                position=tuple(round(float(v), 4) for v in positions[i]),
                velocity=tuple(round(float(v), 4) for v in velocities[i]),
                target=tuple(round(float(v), 4) for v in formation.target_for(i)),
                accel_cmd=tuple(round(float(v), 4) for v in agent.last_cmd),
                formation_error_m=round(float(errors[i]), 4),
                healthy=bool(agent.healthy and agent.battery_ok()),
                fault=agent.fault,
                battery_frac=round(float(agent.battery_frac), 4),
                load_share_n=round(float(shares[i]), 4),
                tether_tension_n=round(float(planner.tensions[i]), 4),
            ).to_dict())

        abort_reason = planner.abort_reason
        if abort_reason and abort_reason != self._last_abort_reason:
            self.safety_violations += 1
            self._last_abort_reason = abort_reason

        frame = FrameSnapshot(
            t=round(float(self.swarm.t), 4),
            phase=phase.value,
            alt_setpoint=round(float(planner.alt_setpoint), 4),
            shape_quality=round(float(formation.shape_quality(positions)), 4),
            min_separation_m=round(min_sep, 4),
            formation_rmse_m=round(float(np.sqrt(np.mean(errors ** 2))), 4),
            load_position=tuple(round(v, 4) for v in load_pos),
            load_mass_carried_kg=round(float(load_mass), 4),
            wind=tuple(round(float(v), 4) for v in wind),
            comm_links=self.comm_links,
            drones=drones,
            abort_reason=abort_reason,
            event=event,
        )
        self.frames.append(frame)
        return frame

    @staticmethod
    def _min_separation(positions) -> float:
        from swarm_drone.geometry import min_pair_distance
        d = min_pair_distance(positions)
        return d if np.isfinite(d) else 0.0

    def attach_points(self):
        """World-frame tether attach points on the payload, for drawing the
        six tether lines. Static per formation radius/attach radius/yaw."""
        return self.swarm.formation.attach_points(self.cfg.lift.attach_radius_m)
