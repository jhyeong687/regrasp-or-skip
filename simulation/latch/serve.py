"""Serve rendered trials and stream fresh MuJoCo runs on the loopback interface."""

import argparse
from functools import partial
from http.server import HTTPServer, SimpleHTTPRequestHandler
import json
from pathlib import Path
import time
from urllib.parse import parse_qs, urlsplit

from run import Bench, CASE_ORDER, SIMULATION
from sim import jpeg


class WorkbenchHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        url = urlsplit(self.path)
        if url.path != "/live":
            return super().do_GET()
        trial = parse_qs(url.query).get("trial", [""])[0]
        case = tuple(trial.split("-"))
        if case not in CASE_ORDER or case[-1] == "wait":
            self.send_error(400, "Choose a recorded wiggle or press condition")
            return
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        start = time.monotonic()

        def send(payload):
            self.wfile.write(json.dumps(payload, separators=(",", ":")).encode() + b"\n")
            self.wfile.flush()

        bench = self.server.bench

        def frame(images, phase):
            sample = bench.sample(phase, include_vectors=True)
            time.sleep(max(0., start + sample["t"] - time.monotonic()))
            send({"sample": sample, "bench": jpeg(images[0]), "cameras": jpeg(images[1])})

        try:
            result, _ = bench.run_trial(*case, frames=frame, include_vectors=True, frame_ms=100)
            send({"result": result})
        except (BrokenPipeError, ConnectionResetError):
            return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", type=Path)
    parser.add_argument("--robot-scene", type=Path, default=SIMULATION / "scene.xml")
    parser.add_argument("--port", type=int, default=8774)
    args = parser.parse_args()
    if not (args.recordings / "replays.json").is_file():
        parser.error("Generate the recordings first with run.py --render")
    with HTTPServer(("127.0.0.1", args.port), partial(WorkbenchHandler, directory=args.recordings)) as server:
        server.bench = Bench(args.robot_scene.resolve())
        try:
            print(f"Workbench at http://127.0.0.1:{args.port}/", flush=True)
            server.serve_forever()
        finally:
            server.bench.close()


if __name__ == "__main__":
    main()
