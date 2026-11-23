"""Run each experimental inference in a new ephemeral Codex app-server thread."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import selectors
import subprocess
import tempfile
import time

from evaluate import EFFORT, MODEL


def utc():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def infer(prompt, image, schema, *, timeout=150):
    """Return the requested configuration, turn events, and process wall time."""
    started = utc()
    clock = time.monotonic()
    receipt = {"started_utc": started, "events": [], "requested_model": MODEL,
               "requested_effort": EFFORT}
    with tempfile.TemporaryDirectory(prefix="r7-placement-") as directory:
        command = [os.environ.get("R7_CODEX_BIN", "codex"), "app-server", "-c", f'model="{MODEL}"',
                   "-c", f'model_reasoning_effort="{EFFORT}"']
        with subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                              stderr=subprocess.DEVNULL, cwd=directory) as process:
            try:
                with selectors.DefaultSelector() as selector:
                    selector.register(process.stdout, selectors.EVENT_READ)
                    buffer = b""

                    def send(message):
                        process.stdin.write(json.dumps(message).encode() + b"\n")
                        process.stdin.flush()

                    def receive():
                        nonlocal buffer
                        while b"\n" not in buffer:
                            remaining = timeout - (time.monotonic() - clock)
                            if remaining <= 0 or not selector.select(remaining):
                                raise TimeoutError(f"Codex turn exceeded {timeout:g} s")
                            block = process.stdout.read1(65536)
                            if not block:
                                raise RuntimeError("Codex app-server closed stdout")
                            buffer += block
                        line, buffer = buffer.split(b"\n", 1)
                        return json.loads(line)

                    def response(request_id):
                        while True:
                            event = receive()
                            if event.get("id") == request_id:
                                if "error" in event:
                                    raise RuntimeError(str(event["error"]))
                                return event["result"]

                    send({"id": 1, "method": "initialize", "params": {"clientInfo": {
                        "name": "r7-stacking-experiment", "version": "1.0"}}})
                    response(1)
                    send({"method": "initialized"})
                    send({"id": 2, "method": "thread/start", "params": {
                        "model": MODEL, "ephemeral": True, "cwd": directory,
                        "sandbox": "read-only", "approvalPolicy": "never",
                        "config": {"model_reasoning_effort": EFFORT},
                        "baseInstructions": "You are evaluating a tabletop robotic manipulation task. "
                        "Use only the supplied observations and return the requested JSON. "
                        "Do not invoke tools, inspect files, or run commands."}})
                    settings = response(2)
                    receipt["runtime"] = {key: settings.get(key) for key in
                                          ("model", "reasoningEffort", "modelProvider")}
                    receipt["thread_id"] = settings["thread"]["id"]
                    if (settings.get("model"), settings.get("reasoningEffort")) != (MODEL, EFFORT):
                        raise RuntimeError(f"Requested model configuration unavailable: {receipt['runtime']}")
                    send({"id": 3, "method": "turn/start", "params": {
                        "threadId": receipt["thread_id"], "model": MODEL, "effort": EFFORT,
                        "outputSchema": schema, "input": [
                            {"type": "text", "text": prompt},
                            {"type": "localImage", "path": str(Path(image).resolve())}]}})
                    for event in iter(receive, None):
                        method = event.get("method")
                        params = event.get("params", {})
                        if event.get("id") == 3 and "error" in event:
                            raise RuntimeError(str(event["error"]))
                        if method == "turn/started":
                            receipt["turn_id"] = params["turn"]["id"]
                        elif method == "error":
                            receipt["turn_error"] = params.get("error")
                        elif method == "thread/tokenUsage/updated":
                            receipt["usage"] = params["tokenUsage"]
                            receipt["events"].append(event)
                        elif method in ("model/rerouted", "model/reroutedForTurn"):
                            receipt["events"].append(event)
                            raise RuntimeError("The server rerouted the requested model")
                        elif method == "item/completed":
                            item = params["item"]
                            if item["type"] == "agentMessage":
                                receipt["events"].append(event)
                                receipt["response"] = json.loads(item["text"])
                            elif item["type"] not in ("userMessage", "reasoning", "plan"):
                                raise RuntimeError(f"Unexpected tool or item in policy turn: {item['type']}")
                        elif method == "turn/completed":
                            receipt["turn_id"] = params["turn"]["id"]
                            receipt["turn_status"] = params["turn"]["status"]
                            if receipt["turn_status"] != "completed" or "response" not in receipt:
                                if params["turn"].get("error") is not None:
                                    receipt["turn_error"] = params["turn"]["error"]
                                raise RuntimeError(f"Codex turn status: {receipt['turn_status']}")
                            break
            except (OSError, ValueError, KeyError, RuntimeError) as error:
                receipt["error"] = f"{type(error).__name__}: {error}"
            finally:
                receipt["wall_s"] = round(time.monotonic() - clock, 3)
                receipt["finished_utc"] = utc()
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
    return receipt
