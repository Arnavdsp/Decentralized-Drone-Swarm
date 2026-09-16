"""
Local, zero-cost real-time server for the browser-based 3D viewer.

Two plain-stdlib-plus-one-package pieces, no cloud, no paid anything:

  * a ``ThreadingHTTPServer`` serving the static files in ``web/`` (plain
    HTML/CSS/JS, three.js pulled from a public CDN by the browser itself);
  * a ``websockets`` server that runs a scenario and streams one JSON
    ``FrameSnapshot`` per control tick, paced to real time (or faster/slower
    via a speed multiplier the page's UI controls).

The physics/control loop itself is exactly ``sim_viz.scenarios.run_scenario``'s
logic, just yielding to the event loop between ticks instead of blocking.
"""
from __future__ import annotations

import asyncio
import inspect
import json
import os
import threading
import webbrowser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import websockets

from .metrics import compute_summary
from .scenarios import SCENARIOS

WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web")


def _filter_kwargs(fn, params: dict) -> dict:
    """Keep only the keys ``fn`` actually accepts, so a stray UI field never
    crashes a scenario builder."""
    sig = inspect.signature(fn)
    return {k: v for k, v in params.items() if k in sig.parameters}


async def _stream_scenario(ws, run, dt: float, speed: float, stop_event: asyncio.Event):
    if run.rejected:
        await ws.send(json.dumps({"type": "rejected",
                                   "feasibility": run.feasibility_report}))
        return

    pending = sorted(run.events, key=lambda e: e[0])
    fired = set()
    steps = int(run.duration_s / dt)

    for step in range(steps):
        if stop_event.is_set():
            break
        t = step * dt
        event_name = None
        for idx, (when, name, fn) in enumerate(pending):
            if idx not in fired and t >= when:
                fn(run.adapter)
                fired.add(idx)
                event_name = name

        frame = run.adapter.tick(dt, event=event_name)
        await ws.send(json.dumps({"type": "frame", "data": frame.to_dict()}))

        if frame.phase in ("landing", "abort") and \
                max(d["position"][2] for d in frame.drones) < 0.05 and frame.t > 0.5:
            break
        await asyncio.sleep(dt / max(speed, 1e-6))

    await ws.send(json.dumps({"type": "done", "summary": compute_summary(run)}))


async def _handler(ws):
    stop_event = asyncio.Event()
    task = None
    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            cmd = msg.get("cmd")

            if cmd == "start":
                if task and not task.done():
                    stop_event.set()
                    await task
                stop_event = asyncio.Event()

                key = msg.get("scenario", "A")
                builder = SCENARIOS.get(key)
                if builder is None:
                    await ws.send(json.dumps({"type": "error",
                                               "message": f"unknown scenario '{key}'"}))
                    continue

                overrides = msg.get("overrides") or {}
                extra = _filter_kwargs(builder, msg.get("params") or {})
                try:
                    run = builder(overrides=overrides, **extra)
                except Exception as exc:  # noqa: BLE001 — surfaced to the UI, not swallowed
                    await ws.send(json.dumps({"type": "error", "message": str(exc)}))
                    continue

                dt = 1.0 / run.cfg.formation.control_hz
                speed = float(msg.get("speed", 1.0))

                await ws.send(json.dumps({
                    "type": "meta",
                    "name": run.name,
                    "description": run.description,
                    "n_drones": run.cfg.formation.n_drones,
                    "config": run.cfg.to_dict(),
                    "comm_links": run.adapter.comm_links,
                    "attach_points": run.adapter.attach_points().tolist(),
                    "feasibility": run.feasibility_report,
                }))
                task = asyncio.create_task(_stream_scenario(ws, run, dt, speed, stop_event))

            elif cmd == "stop":
                stop_event.set()
    finally:
        stop_event.set()
        if task:
            task.cancel()


def _serve_static(port: int):
    handler = partial(SimpleHTTPRequestHandler, directory=WEB_DIR)
    httpd = ThreadingHTTPServer(("127.0.0.1", port), handler)
    httpd.serve_forever()


async def _amain(http_port: int, ws_port: int, open_browser: bool):
    threading.Thread(target=_serve_static, args=(http_port,), daemon=True).start()
    async with websockets.serve(_handler, "127.0.0.1", ws_port):
        url = f"http://127.0.0.1:{http_port}/?ws=ws://127.0.0.1:{ws_port}"
        print(f"Swarm visualizer running:")
        print(f"  UI        -> {url}")
        print(f"  Telemetry -> ws://127.0.0.1:{ws_port}")
        if open_browser:
            webbrowser.open(url)
        await asyncio.Future()  # run forever


def main(http_port: int = 8000, ws_port: int = 8765, open_browser: bool = True):
    try:
        asyncio.run(_amain(http_port, ws_port, open_browser))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
