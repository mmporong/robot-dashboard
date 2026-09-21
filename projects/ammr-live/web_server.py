#!/usr/bin/env python3
"""Read-only local gateway for the AMMR operations dashboard."""

from __future__ import annotations

import argparse
import http.client
import json
import re
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import SplitResult, urlsplit


HERE = Path(__file__).resolve().parent
DEFAULT_BIND = "127.0.0.1"
DEFAULT_PORT = 8090
DEFAULT_BACKEND_PORT = 18091
BACKEND_HOST = "127.0.0.1"
SOCKET_TIMEOUT_SECONDS = 5.0
MAX_API_BODY_BYTES = 4 * 1024 * 1024

STATIC_FILES = {
    "/": ("dashboard.html", "text/html; charset=utf-8"),
    "/dashboard.js": ("dashboard.js", "text/javascript; charset=utf-8"),
}
PROXY_PATHS = {
    "/api/state",
    "/api/map.png",
    "/health",
    "/stream/color.mjpg",
    "/stream/depth.mjpg",
}
STREAM_PATHS = {"/stream/color.mjpg", "/stream/depth.mjpg"}
PASSTHROUGH_HEADERS = ("Content-Type", "Content-Length", "Content-Disposition")
PUBLIC_REPOSITORY = "mmporong/robot-dashboard"
PUBLIC_FILE_NAMES = ("dashboard.html", "dashboard.js", "web_server.py")
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")


def valid_port(value: str) -> int:
    """Parse a user-facing TCP port and reject zero or out-of-range values."""
    try:
        port = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("port must be an integer") from exc
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return port


def _public_version() -> dict[str, Any]:
    """Return validated release metadata without unrelated JSON fields."""
    version_file = HERE / "version.json"
    if not version_file.is_file():
        return {"version": "development"}
    try:
        document = json.loads(version_file.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return {"version": "development"}
    if not isinstance(document, dict):
        return {"version": "development"}
    version = document.get("version")
    if not isinstance(version, (str, int, float)) or isinstance(version, bool):
        return {"version": "development"}
    public: dict[str, Any] = {"version": version}
    if document.get("repository") == PUBLIC_REPOSITORY:
        public["repository"] = PUBLIC_REPOSITORY
    files = document.get("files")
    if isinstance(files, dict):
        validated_files = {
            name: digest
            for name in PUBLIC_FILE_NAMES
            if isinstance((digest := files.get(name)), str)
            and SHA256_PATTERN.fullmatch(digest) is not None
        }
        if len(validated_files) == len(PUBLIC_FILE_NAMES):
            public["files"] = validated_files
    return public


def _safe_target(raw_target: str) -> SplitResult | None:
    """Accept origin-form request targets only."""
    try:
        target = urlsplit(raw_target)
    except ValueError:
        return None
    if target.scheme or target.netloc or target.fragment:
        return None
    return target


def make_handler(backend_port: int) -> type[BaseHTTPRequestHandler]:
    """Create a gateway handler pinned to one localhost backend port."""
    if not 1 <= backend_port <= 65535:
        raise ValueError("backend_port must be between 1 and 65535")

    class GatewayHandler(BaseHTTPRequestHandler):
        server_version = "AMMRGateway/1.0"
        sys_version = ""
        protocol_version = "HTTP/1.1"

        def setup(self) -> None:
            super().setup()
            self.connection.settimeout(SOCKET_TIMEOUT_SECONDS)

        def log_message(self, format: str, *args: object) -> None:
            # Avoid request contents and stack traces in normal local operation.
            return

        def do_GET(self) -> None:
            target = _safe_target(self.path)
            if target is None:
                self._json_error(400, "invalid request target")
                return

            if target.path in STATIC_FILES and not target.query:
                filename, content_type = STATIC_FILES[target.path]
                self._serve_static(filename, content_type)
            elif target.path == "/version" and not target.query:
                self._send_json(200, _public_version())
            elif target.path in PROXY_PATHS:
                self._proxy(target)
            else:
                self._json_error(404, "not found")

        def do_POST(self) -> None:
            self._method_not_allowed()

        def do_PUT(self) -> None:
            self._method_not_allowed()

        def do_PATCH(self) -> None:
            self._method_not_allowed()

        def do_DELETE(self) -> None:
            self._method_not_allowed()

        def do_OPTIONS(self) -> None:
            self._method_not_allowed()

        def do_HEAD(self) -> None:
            self._method_not_allowed()

        def _method_not_allowed(self) -> None:
            self.close_connection = True
            self._send_json(405, {"error": "method not allowed"}, {"Allow": "GET"})

        def _serve_static(self, filename: str, content_type: str) -> None:
            try:
                body = (HERE / filename).read_bytes()
            except OSError:
                self._json_error(500, "dashboard asset unavailable")
                return
            self._send_bytes(200, body, content_type)

        def _proxy(self, target: SplitResult) -> None:
            upstream_target = target.path
            if target.query:
                upstream_target += "?" + target.query

            connection = http.client.HTTPConnection(
                BACKEND_HOST, backend_port, timeout=SOCKET_TIMEOUT_SECONDS
            )
            response: http.client.HTTPResponse | None = None
            try:
                connection.request(
                    "GET",
                    upstream_target,
                    headers={"Accept": "*/*", "Connection": "close"},
                )
                response = connection.getresponse()
                if 300 <= response.status < 400:
                    self._json_error(502, "backend redirect denied")
                    return
                if target.path in STREAM_PATHS:
                    self._relay_stream(response)
                else:
                    self._relay_bounded(response)
            except (ConnectionError, TimeoutError, OSError, http.client.HTTPException):
                if not self.wfile.closed:
                    self._json_error(503, "backend unavailable")
            finally:
                if response is not None:
                    response.close()
                connection.close()

        def _relay_bounded(self, response: http.client.HTTPResponse) -> None:
            length = response.getheader("Content-Length")
            if length is not None:
                try:
                    if int(length) > MAX_API_BODY_BYTES:
                        self._json_error(502, "backend response too large")
                        return
                except ValueError:
                    self._json_error(502, "invalid backend response")
                    return

            body = response.read(MAX_API_BODY_BYTES + 1)
            if len(body) > MAX_API_BODY_BYTES:
                self._json_error(502, "backend response too large")
                return
            try:
                self.send_response(response.status)
                self._copy_upstream_headers(response, len(body))
                self.end_headers()
                self.wfile.write(body)
            except (
                BrokenPipeError,
                ConnectionResetError,
                TimeoutError,
                OSError,
                http.client.HTTPException,
            ):
                self.close_connection = True

        def _relay_stream(self, response: http.client.HTTPResponse) -> None:
            try:
                self.send_response(response.status)
                self._copy_upstream_headers(response, None)
                self.end_headers()
                while True:
                    chunk = response.read1(64 * 1024)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (
                BrokenPipeError,
                ConnectionResetError,
                TimeoutError,
                OSError,
                http.client.HTTPException,
            ):
                self.close_connection = True

        def _copy_upstream_headers(
            self, response: http.client.HTTPResponse, body_length: int | None
        ) -> None:
            for header in PASSTHROUGH_HEADERS:
                if header == "Content-Length":
                    continue
                value = response.getheader(header)
                if value is not None:
                    self.send_header(header, value)
            if body_length is not None:
                self.send_header("Content-Length", str(body_length))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Connection", "close")
            self.close_connection = True

        def _json_error(self, status: int, message: str) -> None:
            self._send_json(status, {"error": message})

        def _send_json(
            self,
            status: int,
            document: dict[str, Any],
            extra_headers: dict[str, str] | None = None,
        ) -> None:
            body = json.dumps(document, ensure_ascii=False, separators=(",", ":")).encode(
                "utf-8"
            )
            self._send_bytes(
                status, body, "application/json; charset=utf-8", extra_headers
            )

        def _send_bytes(
            self,
            status: int,
            body: bytes,
            content_type: str,
            extra_headers: dict[str, str] | None = None,
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            if extra_headers:
                for name, value in extra_headers.items():
                    self.send_header(name, value)
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                self.close_connection = True

    return GatewayHandler


class GatewayServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def create_server(bind: str, port: int, backend_port: int) -> GatewayServer:
    return GatewayServer((bind, port), make_handler(backend_port))


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default=DEFAULT_BIND)
    parser.add_argument("--port", type=valid_port, default=DEFAULT_PORT)
    parser.add_argument(
        "--backend-port", type=valid_port, default=DEFAULT_BACKEND_PORT
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    server = create_server(args.bind, args.port, args.backend_port)

    def stop(_signum: int, _frame: object) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    for name in ("SIGINT", "SIGTERM"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), stop)

    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
