"""Serve the conveyor dashboard and stream the selected policy's MuJoCo trial."""

import argparse
from functools import partial
from http.server import HTTPServer, SimpleHTTPRequestHandler
import json
from pathlib import Path
import time
from urllib.parse import parse_qs, urlsplit

from evaluate import force_rule, make_observation, read_decision
from run import Conveyor, HERE, PARTS, PITCH, POINTS, RATES
from sim import jpeg


class Dashboard(SimpleHTTPRequestHandler):
    def do_GET(self):
        url = urlsplit(self.path)
        if url.path == "/setup":
            return self.send_json({"points": POINTS, "parts": [part.name for part in PARTS], "rates": RATES})
        if url.path != "/live":
            return super().do_GET()
        query = parse_qs(url.query)
        part_name = query.get("part", [""])[0]
        speed = query.get("speed", [""])[0]
        policy = query.get("policy", [""])[0]
        part = next((part for part in PARTS if part.name == part_name), None)
        if part is None or speed not in RATES or policy not in ("force_rule", "astra"):
            return self.send_error(400, "Choose a part, speed, and policy")
        choice = None
        inference_s = None
        if policy == "astra":
            calls = self.server.calls
            if calls is None or not calls.is_file():
                return self.send_error(409, "Astra decisions are not available yet")
            record = json.loads(calls.read_text()).get(f"{part_name}-{speed}", {})
            choice = record.get("choice")
            inference_s = record.get("wall_s")
            if choice is None:
                return self.send_error(409, "Astra is still evaluating this condition")
        rate = RATES[speed]
        bench = Conveyor(self.server.scene, part)
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        start = time.monotonic()
        decision_time = None

        def send(payload):
            self.wfile.write(json.dumps(payload, separators=(",", ":")).encode()+b"\n")
            self.wfile.flush()

        def frame(images):
            sample = bench.trace[-1]
            time.sleep(max(0.,start + sample["t"] - time.monotonic()))
            send({"sample": sample, "bench": jpeg(images[0]), "cameras": jpeg(images[1]),
                  "remaining_s": None if decision_time is None else max(0., PITCH/rate-(sample["t"]-decision_time))})

        try:
            bench.frame_sink = frame
            touches = bench.first_probe()
            decision_time = bench.data.time
            choice = choice or force_rule(make_observation(touches, rate))
            send({"choice": choice, "original_inference_s": inference_s})
            placed = bench.execute(*read_decision(choice), rate)
            elapsed = bench.data.time-decision_time
            send({"result": {"placed": placed, "on_time": placed and elapsed <= PITCH/rate,
                             "time_s": round(elapsed, 3)}})
        except (BrokenPipeError, ConnectionResetError):
            return
        finally:
            bench.close()

    def send_json(self, payload):
        content = json.dumps(payload).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--robot-scene", type=Path, default=HERE.parent / "scene.xml")
    parser.add_argument("--calls", type=Path)
    parser.add_argument("--port", type=int, default=8776)
    args = parser.parse_args()
    with HTTPServer(("127.0.0.1", args.port), partial(Dashboard, directory=args.directory)) as server:
        server.scene = args.robot_scene.resolve()
        server.calls = args.calls.resolve() if args.calls else None
        print(f"http://127.0.0.1:{args.port}/", flush=True)
        server.serve_forever()


if __name__ == "__main__":
    main()
