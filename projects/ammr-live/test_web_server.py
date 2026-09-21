#!/usr/bin/env python3

from __future__ import annotations

import http.client
import json
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest import mock

import web_server


MAP_BYTES = b"\x89PNG\r\n\x1a\nmock-map"


class MockBackendHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, format: str, *args: object) -> None:
        return

    def do_GET(self) -> None:
        if self.path == "/api/state?redirect=1":
            self.send_response(302)
            self.send_header("Location", "http://example.invalid/secret")
            self.send_header("Content-Length", "0")
            self.end_headers()
        elif self.path == "/api/state?large=1":
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(web_server.MAX_API_BODY_BYTES + 1))
            self.end_headers()
        elif self.path.startswith("/api/state"):
            body = json.dumps({"request_target": self.path}).encode()
            self._send(200, body, "application/json")
        elif self.path.startswith("/api/map.png"):
            self._send(200, MAP_BYTES, "image/png")
        elif self.path == "/health":
            self._send(200, b'{"status":"ok"}', "application/json")
        elif self.path == "/stream/color.mjpg?broken=1":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Content-Length", "100")
            self.end_headers()
            self.wfile.write(b"partial-stream")
            self.close_connection = True
        elif self.path.startswith("/stream/color.mjpg"):
            self._send(200, b"--frame\r\nmock-jpeg\r\n", "multipart/x-mixed-replace; boundary=frame")
        else:
            self._send(404, b"missing", "text/plain")

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class GatewayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.backend = ThreadingHTTPServer(("127.0.0.1", 0), MockBackendHandler)
        cls.backend.daemon_threads = True
        cls.backend_thread = threading.Thread(
            target=cls.backend.serve_forever, daemon=True
        )
        cls.backend_thread.start()

        cls.gateway = web_server.create_server(
            "127.0.0.1", 0, cls.backend.server_address[1]
        )
        cls.gateway_thread = threading.Thread(
            target=cls.gateway.serve_forever, daemon=True
        )
        cls.gateway_thread.start()
        cls.port = cls.gateway.server_address[1]

    @classmethod
    def tearDownClass(cls) -> None:
        cls.gateway.shutdown()
        cls.gateway.server_close()
        cls.backend.shutdown()
        cls.backend.server_close()
        cls.gateway_thread.join(timeout=2)
        cls.backend_thread.join(timeout=2)

    def request(self, method: str, target: str) -> tuple[int, dict[str, str], bytes]:
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=2)
        connection.request(method, target)
        response = connection.getresponse()
        body = response.read()
        result = response.status, dict(response.getheaders()), body
        connection.close()
        return result

    def test_static_ui_loads_without_contacting_backend(self) -> None:
        unused = ThreadingHTTPServer(("127.0.0.1", 0), MockBackendHandler)
        unused_port = unused.server_address[1]
        unused.server_close()
        server = web_server.create_server("127.0.0.1", 0, unused_port)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            connection = http.client.HTTPConnection(
                "127.0.0.1", server.server_address[1], timeout=2
            )
            connection.request("GET", "/")
            response = connection.getresponse()
            status, headers, body = (
                response.status,
                dict(response.getheaders()),
                response.read(),
            )
            self.assertEqual(status, 200)
            self.assertIn(b"AMMR", body)
            self.assertEqual(headers["Cache-Control"], "no-store")

            connection.request("GET", "/dashboard.js")
            response = connection.getresponse()
            status, headers, body = (
                response.status,
                dict(response.getheaders()),
                response.read(),
            )
            self.assertEqual(status, 200)
            self.assertIn(b"/api/state", body)
            self.assertEqual(headers["Cache-Control"], "no-store")
            connection.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_backend_unavailable_returns_json_503(self) -> None:
        unused = ThreadingHTTPServer(("127.0.0.1", 0), MockBackendHandler)
        unused_port = unused.server_address[1]
        unused.server_close()
        server = web_server.create_server("127.0.0.1", 0, unused_port)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            connection = http.client.HTTPConnection(
                "127.0.0.1", server.server_address[1], timeout=2
            )
            connection.request("GET", "/api/state")
            response = connection.getresponse()
            self.assertEqual(response.status, 503)
            self.assertEqual(response.getheader("Content-Type"), "application/json; charset=utf-8")
            self.assertEqual(json.loads(response.read()), {"error": "backend unavailable"})
            connection.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_query_is_preserved_and_json_is_passed_through(self) -> None:
        status, headers, body = self.request("GET", "/api/state?a=1%202&b=%2F")
        self.assertEqual(status, 200)
        self.assertEqual(headers["Content-Type"], "application/json")
        self.assertEqual(
            json.loads(body), {"request_target": "/api/state?a=1%202&b=%2F"}
        )

    def test_map_binary_is_passed_through(self) -> None:
        status, headers, body = self.request("GET", "/api/map.png?version=7")
        self.assertEqual(status, 200)
        self.assertEqual(headers["Content-Type"], "image/png")
        self.assertEqual(body, MAP_BYTES)

    def test_redirect_is_denied_without_location_header(self) -> None:
        status, headers, body = self.request("GET", "/api/state?redirect=1")
        self.assertEqual(status, 502)
        self.assertNotIn("Location", headers)
        self.assertEqual(json.loads(body), {"error": "backend redirect denied"})

    def test_api_response_larger_than_four_mib_is_denied(self) -> None:
        status, _, body = self.request("GET", "/api/state?large=1")
        self.assertEqual(status, 502)
        self.assertEqual(json.loads(body), {"error": "backend response too large"})

    def test_mjpeg_stream_is_relayed_without_api_body_limit(self) -> None:
        status, headers, body = self.request("GET", "/stream/color.mjpg?ts=1")
        self.assertEqual(status, 200)
        self.assertEqual(
            headers["Content-Type"], "multipart/x-mixed-replace; boundary=frame"
        )
        self.assertEqual(body, b"--frame\r\nmock-jpeg\r\n")

    def test_broken_stream_does_not_append_a_second_http_response(self) -> None:
        status, _, body = self.request("GET", "/stream/color.mjpg?broken=1")
        self.assertEqual(status, 200)
        self.assertEqual(body, b"partial-stream")
        self.assertNotIn(b"503", body)

    def test_forbidden_paths_absolute_targets_and_post_are_denied(self) -> None:
        for target in ("/etc/passwd", "/../dashboard.html", "/dashboard.html"):
            with self.subTest(target=target):
                status, _, _ = self.request("GET", target)
                self.assertEqual(status, 404)

        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=2)
        connection.request("GET", "http://example.invalid/api/state")
        response = connection.getresponse()
        self.assertEqual(response.status, 400)
        response.read()
        connection.close()

        status, headers, body = self.request("POST", "/api/state")
        self.assertEqual(status, 405)
        self.assertEqual(headers["Allow"], "GET")
        self.assertEqual(json.loads(body), {"error": "method not allowed"})

    def test_version_exposes_only_public_version(self) -> None:
        status, _, body = self.request("GET", "/version")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body), {"version": "development"})

        version_file = mock.Mock()
        version_file.is_file.return_value = True
        version_file.read_text.return_value = (
            '{"version":"2026.09.21","token":"must-not-leak"}'
        )
        with mock.patch.object(web_server, "HERE") as here:
            here.__truediv__.return_value = version_file
            status, _, body = self.request("GET", "/version")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body), {"version": "2026.09.21"})
        self.assertNotIn(b"token", body)

    def test_version_exposes_only_validated_release_metadata(self) -> None:
        digest = "a" * 64
        version_file = mock.Mock()
        version_file.is_file.return_value = True
        version_file.read_text.return_value = json.dumps(
            {
                "repository": "mmporong/robot-dashboard",
                "version": "b" * 40,
                "files": {
                    "dashboard.html": digest,
                    "dashboard.js": digest,
                    "web_server.py": digest,
                    "private.env": "secret",
                },
                "token": "must-not-leak",
            }
        )
        with mock.patch.object(web_server, "HERE") as here:
            here.__truediv__.return_value = version_file
            status, _, body = self.request("GET", "/version")
        self.assertEqual(status, 200)
        self.assertEqual(
            json.loads(body),
            {
                "repository": "mmporong/robot-dashboard",
                "version": "b" * 40,
                "files": {
                    "dashboard.html": digest,
                    "dashboard.js": digest,
                    "web_server.py": digest,
                },
            },
        )
        self.assertNotIn(b"private.env", body)
        self.assertNotIn(b"token", body)

        version_file.read_text.return_value = json.dumps(
            {
                "repository": "private/repository",
                "version": "release",
                "files": {
                    "dashboard.html": digest,
                    "dashboard.js": "not-a-sha256",
                    "web_server.py": digest,
                },
            }
        )
        with mock.patch.object(web_server, "HERE") as here:
            here.__truediv__.return_value = version_file
            _, _, body = self.request("GET", "/version")
        self.assertEqual(json.loads(body), {"version": "release"})

    def test_cli_ports_are_validated(self) -> None:
        args = web_server.parse_args([])
        self.assertEqual((args.bind, args.port, args.backend_port), ("127.0.0.1", 8090, 18091))
        for value in ("0", "65536", "not-a-port"):
            with self.subTest(value=value), self.assertRaises(SystemExit):
                web_server.parse_args(["--port", value])


if __name__ == "__main__":
    unittest.main()
