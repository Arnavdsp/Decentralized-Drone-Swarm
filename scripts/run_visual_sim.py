#!/usr/bin/env python3
"""
Launch the interactive 3D swarm visualizer.

    python scripts/run_visual_sim.py
    python scripts/run_visual_sim.py --http-port 8080 --ws-port 8899
    python scripts/run_visual_sim.py --no-browser

Opens a browser tab serving ``web/index.html``, which connects over a
WebSocket to a local server streaming live telemetry from the *existing*
``swarm_drone`` control stack (see ``sim_viz/``). Everything runs on
localhost — no cloud service, no paid software, no GPU required.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim_viz.server import main  # noqa: E402


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--ws-port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()
    main(http_port=args.http_port, ws_port=args.ws_port, open_browser=not args.no_browser)
