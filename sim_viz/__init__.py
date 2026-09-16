"""
sim_viz — a visualization and scenario layer built *around* the existing
``swarm_drone`` control stack.

Nothing in here touches ``DroneAgent``, ``HexFormation``, ``LiftPlanner``,
``SwarmCoordinator`` or the backends. Those remain the single source of
control-law truth, exactly as shipped. This package only:

  1. observes them after each tick (``adapter.SimulationAdapter``),
  2. drives them through the five demo scenarios (``scenarios``),
  3. scores the resulting flight logs (``metrics``), and
  4. streams the observations to a browser-based 3D renderer (``server``).

See ``docs/SIMULATOR.md`` for the full architecture writeup.
"""
from .adapter import SimulationAdapter, FrameSnapshot, DroneSnapshot

__all__ = ["SimulationAdapter", "FrameSnapshot", "DroneSnapshot"]
