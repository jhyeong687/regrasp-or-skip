"""Serve the read-only local view of an in-progress or completed pick run."""

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class Dashboard(SimpleHTTPRequestHandler):
    page = Path(__file__).with_name("index.html")

    def do_GET(self):
        if self.path.split("?", 1)[0] == "/":
            page = self.page.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)
        else:
            super().do_GET()

    def list_directory(self, path):
        self.send_error(404)
        return None

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--port", type=int, default=62653)
    parser.add_argument("--page", type=Path, default=Dashboard.page)
    args = parser.parse_args()
    Dashboard.page = args.page.resolve()
    handler = partial(Dashboard, directory=args.out.resolve())
    with ThreadingHTTPServer(("127.0.0.1", args.port), handler) as server:
        print(f"http://127.0.0.1:{server.server_port}/", flush=True)
        server.serve_forever()
