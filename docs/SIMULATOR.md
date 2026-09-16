# Interactive 3D Simulator for the Decentralized Drone Swarm

This document is the deliverable writeup: what was analyzed, what was built,
how to run it, how it maps back onto the existing control stack, what it
actually proved when run, and what it doesn't do.

---

## 1. Repository architecture analysis (Phase 1)

### File map

```
swarm_drone/
  config.py      SwarmConfig / DroneSpec / FormationConfig / LiftConfig / CommsConfig
  geometry.py    hexagon math, ring topology, clamping — pure functions, no state
  agent.py       DroneAgent: one drone's backend + link + health, on hardware or in sim
  backends.py    BaseBackend interface; SimBackend (point-mass+drag); MavlinkBackend
  comms.py       BaseLink interface; LoopbackLink (sim/tests); UdpMeshLink (hardware)
  formation.py   HexFormation: slot assignment + the decentralized control law
  load_lift.py   LiftPhase state machine, tether/load equilibrium, feasibility, LiftPlanner
  swarm.py       SwarmCoordinator: ticks all agents, runs the planner, logs history
scripts/
  run_sim.py     Headless CLI: build 6 drones, run a mission, optional matplotlib plot
```

### Execution path of `scripts/run_sim.py`

1. `SwarmConfig()` — defaults for a 6-drone hexagon carrying a 4 kg payload.
2. `feasibility(...)` — checks capacity before anything flies.
3. `build_swarm(cfg)` — scatters 6 `DroneAgent`s (each with a `SimBackend` and a
   `LoopbackLink` sharing one in-process `bus`) on the ground around the origin.
4. `SwarmCoordinator(cfg, agents)` — owns one `HexFormation` and one `LiftPlanner`.
5. `swarm.start_lift(cruise_target)` → `LiftPlanner.start()`: re-checks
   feasibility, binds drones to hexagon slots (`HexFormation.bind`), arms every
   agent, enters `ARMING`.
6. `swarm.tick(dt)` in a loop until `LANDING`/`ABORT` at ground level:
   - reads `positions()`/`velocities()` from every agent's backend,
   - aborts if any agent is unhealthy or under-battery,
   - `LiftPlanner.step()` advances the mission state machine and the shared
     altitude setpoint,
   - `HexFormation.commands()` computes one acceleration per drone from four
     terms (formation, consensus, integral, damping) plus avoidance and an
     altitude-sync term,
   - `LiftPlanner.current_shares()` solves the tether/load equilibrium for the
     vertical newtons each drone carries right now,
   - each `DroneAgent.apply()` clamps the command to its airframe envelope and
     sends it to `SimBackend.send_acceleration()`, which integrates one step of
     point-mass-with-drag-and-tether-load physics.

### How the pieces interact

`SwarmConfig` is pure data, read by everything else. `DroneAgent` is a thin
shell around a `BaseBackend` (physics or MAVLink) and a `BaseLink` (mesh or
loopback) — it owns no control law itself. `HexFormation` is the control law:
a *virtual structure* (center + yaw + radius) that every drone derives its own
target from, using only its own state and its two ring neighbors
(`geometry.ring_neighbours`) — there is no leader. `LiftPlanner` owns the
mission state machine and, through `load_lift.solve_load_equilibrium`, the
quasi-static physics of six unilateral-spring tethers meeting one hanging load.
`SwarmCoordinator` is the only thing that sees all six agents at once; on
hardware, each Pi would run its own coordinator instance with `local_id` set
and use the mesh link in place of ground truth for its neighbors.

### What was directly reusable

Everything. Every file above is used unmodified. `SimBackend` already does
exactly what a "physics/rendering engine" adapter needs to read from: exposes
`state()` for position/velocity, and already models tether drag as reduced
available thrust — no more physics needed for a visualization.

### What had to be adapted for a graphical simulator

Nothing in the control stack — only *observation*. Two data points a renderer
needs are not stored anywhere upstream:

1. **Payload world position.** `LiftPlanner.current_shares()` solves for
   `load_z` internally but nothing keeps a time series of it, and the payload's
   horizontal position (the swarm centroid) isn't stored either.
2. **Per-tick health/tension/battery.** `SwarmCoordinator.history` logs
   positions, phase, altitude setpoint, shape quality, and load shares — but
   not tether *tension* (a different number from vertical share), not health
   flags, not battery, not commanded acceleration, and not the wind vector.

Both are solved by the new `sim_viz.adapter.SimulationAdapter`, which reads
these values off already-public state (`planner.load_z`, `planner.tensions`,
`agent.healthy`, `agent.battery_frac`, `agent.last_cmd`,
`backend.wind`) after each tick — no upstream code changed.

### Assumptions in the current point-mass physics

- `SimBackend` is a double integrator with linear drag and no attitude
  dynamics — realistic enough to exercise the formation/lift logic, not a
  flight-dynamics model.
- The payload is **not** an independently integrated rigid body. Its position
  is solved as a quasi-static equilibrium every tick
  (`solve_load_equilibrium`'s bisection on `load_z`), assuming it hangs
  directly under the swarm's horizontal centroid. This is a documented,
  intentional simplification in the existing code (see the module docstring
  in `load_lift.py`) — the simulator visualizes exactly this assumption,
  it doesn't add payload inertia that isn't there.
- Wind is a constant acceleration term added in `SimBackend.send_acceleration`,
  uniform across all six drones (no per-drone gust variation, no altitude
  shear).

### Bugs / missing interfaces found

None that block visualization — see "what had to be adapted" above; those
were gaps in *logging*, not bugs in the control law. The one thing worth
flagging for anyone tuning gains: `HexFormation.__init__` already warns at
construction time if `k_consensus` is set high enough to cancel
`k_formation` on the ring's alternating mode (see `stability_margin()`); the
visualizer's parameter panel does not re-validate this, so a `RuntimeWarning`
from the existing code is the only guard if you push the sliders somewhere
unstable. That's the existing repo's own safety check working correctly; the
simulator would print it to whatever console started the server.

---

## 2. Simulation architecture (Phase 2)

```
   swarm_drone (unmodified)
   SwarmCoordinator.tick()
            |
            v
   sim_viz.adapter.SimulationAdapter     <-- read-only observer
            |
            v
   sim_viz.scenarios.ScenarioRun         <-- what to inject, when
            |
            v
   sim_viz.server (asyncio + websockets) <-- real-time JSON telemetry
            |
            v
   web/ (three.js, plain JS)             <-- 3D render + debug HUD + params
```

`sim_viz.metrics` sits beside the server, consuming the same `ScenarioRun`
either headlessly (for the CSV/JSON export and the validation script) or at
the end of a live run (the "Scenario complete" summary in the UI).

No file under `swarm_drone/` was changed. Every new file lives under
`sim_viz/`, `web/`, or `scripts/run_visual_sim.py` and `tests/test_sim_viz.py`.

### Files added

```
sim_viz/__init__.py     public re-exports
sim_viz/adapter.py       SimulationAdapter, DroneSnapshot, FrameSnapshot
sim_viz/scenarios.py     scenario_normal/wind/failure/overload/disturbance, run_scenario()
sim_viz/metrics.py       compute_summary(), export_json(), export_csv()
sim_viz/server.py        asyncio websockets server + static file server
web/index.html           page layout: scenario panel, 3D viewport, telemetry panel
web/style.css            dark UI styling
web/app.js               three.js scene, WebSocket client, overlay + param logic
scripts/run_visual_sim.py  CLI launcher
tests/test_sim_viz.py    12 tests: adapter fidelity, each scenario, metrics export
```

---

## 3. Installation

```bash
# from the repo root
python3 -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate

pip install numpy matplotlib pyyaml pytest websockets
```

`websockets` is the only new dependency versus the existing `requirements.txt`
(everything else — `numpy`, `matplotlib`, `pyyaml`, `pytest` — is already
listed there). No GPU, no paid license, no cloud service. The browser fetches
three.js from a public CDN (`unpkg.com`) at load time — no bundler, no local
install of a 3D engine.

## 4. Running it

```bash
python scripts/run_visual_sim.py
```

This opens `http://127.0.0.1:8000/` in your default browser and starts a
telemetry socket on `ws://127.0.0.1:8765`. Pick a scenario (A–E) in the left
panel, adjust parameters if you like, and press **Run scenario**. The right
panel has overlay toggles (communication links, tethers, target slots,
formation center, acceleration/velocity vectors, trajectories) and live
per-drone telemetry cards you can click to highlight a drone in the 3D view.

Useful flags:

```bash
python scripts/run_visual_sim.py --http-port 8080 --ws-port 8899
python scripts/run_visual_sim.py --no-browser   # print the URL instead of opening one
```

For headless runs (no browser, e.g. to generate the CSV/JSON metrics or to
smoke-test a scenario in CI):

```python
from sim_viz.scenarios import SCENARIOS, run_scenario
from sim_viz.metrics import compute_summary, export_json, export_csv

run = run_scenario(SCENARIOS["B"](wind_speed=1.0, wind_start_s=5.0))
print(compute_summary(run))
export_json(run, "out/wind_run.json")
export_csv(run, "out/wind_run.csv")
```

The original headless CLI still works exactly as before and is unaffected:

```bash
python scripts/run_sim.py --wind 0.8 --plot out.png
```

---

## 5. How the simulation maps to the original repository

| What you see in the browser              | Where it actually comes from                                   |
|-------------------------------------------|------------------------------------------------------------------|
| Drone position/orientation                | `SimBackend.state()` via `DroneAgent.position` / `.velocity`     |
| Drone color (blue/red)                    | `agent.healthy and agent.battery_ok()`                            |
| Target slot marker                        | `HexFormation.target_for(i)`                                     |
| Tether line                                | drone position → (payload position + static hex attach offset)   |
| Payload position                          | swarm centroid (xy) + `LiftPlanner.load_z`, both already solved by `solve_load_equilibrium` inside `current_shares()` |
| Tether tension readout                    | `LiftPlanner.tensions[i]`, set by the same call                  |
| Load-share readout                        | `LiftPlanner.current_shares()` (vertical N per drone)             |
| Communication links                        | `geometry.ring_neighbours` topology (static, drawn between the drones' *current* positions) |
| Mission phase banner                       | `LiftPlanner.phase.value`                                         |
| Formation RMSE / shape quality             | `HexFormation.errors()` / `.shape_quality()`                      |
| Min separation                             | `geometry.min_pair_distance`                                     |
| Battery %                                  | `agent.battery_frac`                                              |
| Wind vector readout                        | `backend.wind` (set via `SimulationAdapter.set_wind`)             |

Nothing here is invented — every number the UI displays is read straight off
an existing public attribute or method.

## 6. How each swarm algorithm appears in the simulation

- **Decentralized formation control (virtual structure):** every drone's
  cone flies toward its own yellow target ring; the ring itself is the same
  for everyone, so the hexagon holds shape even before you'd notice any
  single drone "leading."
- **Local neighbor consensus:** toggle "Communication links" — only the ring
  edges are drawn, matching exactly what `HexFormation.command()` actually
  reads (`self.neighbours[i]`, not all six drones).
- **Cooperative lift / load sharing:** the per-drone telemetry cards show
  live load-share newtons; in Scenario B/C/E you can watch one card's share
  climb while another's drops as the ring tilts.
- **Tether physics:** tether lines visibly slacken pre-`TENSIONING` and pull
  taut as `tension_frac` ramps to 1.0; the payload icosahedron only appears
  once `load_mass_carried_kg > 0`.
- **Wind rejection:** Scenario B's formation-RMSE readout spikes on wind-on,
  then settles as the integral term (`k_integral`) builds a standing
  acceleration against it — visible directly in the metric, not just implied.
- **Drone failure / no reassignment:** Scenario C turns one cone red at
  `fail_at_s`, and the mission phase banner flips straight to `ABORT` —
  the swarm does not attempt to redistribute the load, matching
  `SwarmCoordinator.tick()`'s existing behavior exactly.
- **Formation recovery:** Scenario E shoves one drone off its slot before the
  tethers load; watch its formation-error readout spike and decay as the
  consensus + avoidance terms pull it back before `TENSIONING` begins.
- **Overload rejection:** Scenario D never arms — the mission-phase banner
  never leaves "idle" and the feasibility numbers (margin vs. required
  margin) are shown directly in the summary panel.

## 7. Test results

```
$ pytest tests/test_formation.py tests/test_load_lift.py tests/test_swarm.py tests/test_sim_viz.py -q
.............................................................
61 passed in ~22s
```

The 49 pre-existing swarm tests pass unmodified. The 12 new tests
(`tests/test_sim_viz.py`) check:

- the adapter produces **bit-identical** dynamics to calling
  `SwarmCoordinator.tick()` directly (same positions/velocities/phase after
  400 ticks, same random seed) — proof the observer layer changes nothing;
- every `FrameSnapshot` is JSON-serializable;
- the payload only "exists" (nonzero position/mass) once actually carried;
- Scenario A completes and lands; Scenario C aborts with the documented
  reason and never reassigns; Scenario D is rejected with zero frames run;
  Scenario E shows a genuine error spike-then-recovery within the same
  mission phase; Scenario B produces higher mean formation error than the
  calm baseline;
- config overrides (`radius_m`, `payload_mass_kg`, ...) actually reach the
  `SwarmConfig` the scenario runs with;
- `compute_summary()` returns the documented metric keys for both a
  completed and a rejected mission, and CSV/JSON export round-trips.

Manual validation against the original `scripts/run_sim.py --plot` output at
the same seed confirmed identical phase-transition timestamps (e.g.
`t=23.45` entering `cruise` in both), the same feasibility numbers (margin
1.554 for the default 4 kg payload), and the same abort behavior for
`--fail-drone 3 --fail-at 20` (aborts at 15.05s local sim time, reason
`"drone(s) [3] unavailable mid-lift"`).

## 8. Known limitations

- **Payload is quasi-static, not a simulated rigid body.** It has no
  independent inertia or swing dynamics — it snaps to the tether-equilibrium
  solution every tick. This matches the existing `load_lift.py` model
  exactly; it is not a simplification introduced by the visualizer.
- **No live mid-flight parameter tuning.** The parameter panel is
  "configure, then run" — changing a gain restarts the scenario from scratch
  rather than hot-patching a running mission. Real-time gain scheduling would
  require exposing mutable config into `HexFormation`/`LiftPlanner`, which
  the existing classes don't support (their `cfg` is read at construction).
- **Single active run per browser tab.** The server supports one scenario
  stream at a time per WebSocket connection; running two scenarios
  side-by-side needs two browser tabs (each opens its own socket).
- **Vision pipeline is out of scope.** `swarm_drone/vision/` (RT-DETR +
  face matching) is untouched and unused here — this simulator is about the
  formation/lift control stack only, as scoped in the brief.
- **Wind is uniform and constant**, per the existing `SimBackend` model — no
  gusts, shear, or per-drone variation.
- **`n_drones` other than 6** technically flows through the config overrides
  (the formation math is general), but the frontend's default camera framing
  and the hexagon-specific "6 tethers" narrative assume 6. `assign_slots`
  itself is exact up to `n=8` and falls back to a greedy heuristic above
  that, per the existing `formation.py` docstring.

## 9. How to demonstrate this project in a robotics/ML interview

Lead with the architecture, not the graphics: *"The control law — a
decentralized virtual-structure formation controller with ring-topology
consensus, plus a tether-equilibrium load-sharing model — was already
implemented and tested. I built an observability layer around it: an adapter
that turns every control tick into a render-ready snapshot without touching
the control code, a scenario harness that scripts failure/wind/disturbance
injection, and a real-time browser renderer over WebSockets."*

A strong 30–60s recording: start Scenario A and let it reach `CRUISE` (~15s
of sim time at 4-8x playback speed), then switch to Scenario C mid-recording
to show the failure abort, calling out on camera that *"the swarm doesn't try
to invent a recovery the underlying planner doesn't support — it aborts
safely, and the simulator is honest about that rather than dramatizing a
fake save."* That line does more in an interview than any visual: it shows
you understand the difference between what the control stack guarantees and
what a demo animates.

Points worth making if asked follow-ups:

- *"Why didn't you just hardcode the visualization from the physics
  equations?"* — Because the adapter reads the actual `HexFormation` and
  `LiftPlanner` objects; if someone retunes `k_consensus` in `config.py`
  tomorrow, the visualization is correct by construction, not by having been
  re-derived.
- *"How would this run on real hardware?"* — Unchanged: `MavlinkBackend`
  already implements the same `BaseBackend` interface `SimBackend` does;
  swapping backends is a constructor argument, not a rewrite. The simulator
  never assumes `SimBackend` specifically — it drives the swarm through the
  public `SwarmCoordinator` API.
- *"What would you build next?"* — Live gain scheduling (would need
  `HexFormation`/`LiftPlanner` to accept mutable config), per-agent
  wind/gust variation, and a second camera mode that shows the mesh
  network's actual heartbeat/timeout behavior (`comms.BaseLink.lost_peers`)
  rather than the ring topology's static shape.
