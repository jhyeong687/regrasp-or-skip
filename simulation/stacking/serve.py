"""Serve the local R7 monitor with byte ranges for browser video seeking."""

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re


class StackRequest(SimpleHTTPRequestHandler):
    def send_head(self):
        path = Path(self.translate_path(self.path))
        self.remaining = None
        range_header = self.headers.get("Range", "")
        requested_range = re.fullmatch(r"bytes=(\d+)-(\d*)", range_header)
        if path.suffix != ".mp4" or not path.is_file() or not requested_range:
            return super().send_head()

        stream = path.open("rb")
        total = path.stat().st_size
        start = int(requested_range[1])
        end = min(int(requested_range[2] or total - 1), total - 1)
        if start > end:
            stream.close()
            self.send_response(416)
            self.send_header("Content-Range", f"bytes */{total}")
            self.end_headers()
            return None

        self.remaining = end-start+1
        self.send_response(206)
        self.send_header("Content-Type", "video/mp4")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(self.remaining))
        self.send_header("Content-Range", f"bytes {start}-{end}/{total}")
        self.end_headers()
        stream.seek(start)
        return stream

    def copyfile(self, source, outputfile):
        if self.remaining is None:
            return super().copyfile(source, outputfile)
        while self.remaining:
            block = source.read(min(self.remaining, 64 * 1024))
            if not block:
                break
            outputfile.write(block)
            self.remaining -= len(block)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--port", type=int, default=8777)
    args = parser.parse_args()
    with ThreadingHTTPServer(("127.0.0.1", args.port), partial(StackRequest, directory=args.directory)) as server:
        print(f"http://127.0.0.1:{args.port}/", flush=True)
        server.serve_forever()


if __name__ == "__main__":
    main()
