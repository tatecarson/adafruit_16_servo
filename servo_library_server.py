#!/usr/bin/env python3
"""Serve the controller and persist the browser library to ./library.json."""

from __future__ import annotations

import argparse
import json
import os
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from tempfile import NamedTemporaryFile


ROOT = Path(__file__).resolve().parent
LIBRARY_PATH = ROOT / "library.json"
MAX_LIBRARY_BYTES = 512 * 1024


class ServoLibraryHandler(SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, PUT, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        super().end_headers()

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self.end_headers()

    def do_POST(self) -> None:
        self._write_library()

    def do_PUT(self) -> None:
        self._write_library()

    def _write_library(self) -> None:
        path = self.path.split("?", 1)[0]
        if path != "/library.json":
            self.send_error(404, "Only /library.json accepts writes")
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_error(400, "Invalid Content-Length")
            return

        if content_length <= 0:
            self.send_error(400, "Empty request body")
            return
        if content_length > MAX_LIBRARY_BYTES:
            self.send_error(413, "Library JSON is too large")
            return

        raw = self.rfile.read(content_length)
        try:
            parsed = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.send_error(400, f"Invalid JSON: {exc}")
            return

        if parsed.get("schemaVersion") != 1:
            self.send_error(400, "Expected schemaVersion 1")
            return

        canonical = json.dumps(parsed, indent=2, sort_keys=False)
        with NamedTemporaryFile("w", encoding="utf-8", dir=ROOT, delete=False) as tmp:
            tmp.write(canonical)
            tmp.write("\n")
            tmp_path = Path(tmp.name)
        os.replace(tmp_path, LIBRARY_PATH)

        body = json.dumps({"ok": True, "bytes": LIBRARY_PATH.stat().st_size}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve servo_controller.html with file-backed library persistence.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4173)
    args = parser.parse_args()

    os.chdir(ROOT)
    server = ThreadingHTTPServer((args.host, args.port), ServoLibraryHandler)
    url = f"http://{args.host}:{args.port}/servo_controller.html"
    print(f"Serving {ROOT}")
    print(f"Controller: {url}")
    print(f"Library:    {LIBRARY_PATH}")
    server.serve_forever()


if __name__ == "__main__":
    main()
