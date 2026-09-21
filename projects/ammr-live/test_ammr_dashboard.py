import argparse
import json
import math
import pathlib
import sys
import unittest
import threading
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from ammr_dashboard import (TelemetryStore, decode_depth, depth_summary,
                            finite_argument, make_handler, occupancy_preview,
                            scan_summary)  # noqa: E402


class DashboardTelemetryTest(unittest.TestCase):
    def test_scan_summary_filters_invalid_and_reports_sectors(self):
        ranges = [1.0, float("inf"), 0.5, float("nan"), 2.0]
        result = scan_summary(ranges, -math.pi, math.pi / 2, 0.1, 8.0)
        self.assertEqual(result["valid_points"], 3)
        self.assertEqual(result["min_m"], 0.5)
        self.assertEqual(result["front_m"], 0.5)
        self.assertEqual(len(result["points_xy_m"]), 3)

    def test_scan_summary_rotates_reverse_mounted_lidar(self):
        result = scan_summary([1.0], math.pi, 0.1, 0.1, 8.0,
                              yaw_deg=180.0)
        self.assertEqual(result["front_m"], 1.0)
        self.assertEqual(result["points_xy_m"], [[1.0, 0.0]])

    def test_scan_summary_applies_display_origin_translation(self):
        result = scan_summary([1.0], 0.0, 0.1, 0.1, 8.0, x_m=-0.25)
        self.assertEqual(result["front_m"], 0.75)
        self.assertEqual(result["points_xy_m"], [[0.75, 0.0]])

    def test_scan_summary_zero_transform_preserves_existing_behavior(self):
        original = scan_summary([1.0], 0.0, 0.1, 0.1, 8.0)
        explicit = scan_summary([1.0], 0.0, 0.1, 0.1, 8.0,
                                yaw_deg=0.0, x_m=0.0, y_m=0.0)
        self.assertEqual(original, explicit)

    def test_scan_summary_rejects_non_finite_transform(self):
        with self.assertRaises(ValueError):
            scan_summary([1.0], 0.0, 0.1, 0.1, 8.0, yaw_deg=float("nan"))

    def test_cli_transform_parameter_rejects_non_finite_value(self):
        for value in ("nan", "inf", "-inf", "not-a-number"):
            with self.subTest(value=value), self.assertRaises(
                    argparse.ArgumentTypeError):
                finite_argument(value)

    def test_depth_navigation_status_becomes_stale(self):
        now = [10.0]
        store = TelemetryStore(clock=lambda: now[0])
        store.update_depth_navigation(SimpleNamespace(
            data='{"healthy": true, "state": "tracking", "processing_ms": 8.5, '
                 '"age_s": 0.4}'))
        fresh = store.snapshot()["depth_navigation"]
        self.assertTrue(fresh["live"])
        self.assertEqual(fresh["age_s"], 0.0)
        self.assertEqual(fresh["source_age_s"], 0.4)
        now[0] += 2.1
        status = store.snapshot()["depth_navigation"]
        self.assertFalse(status["live"])
        self.assertEqual(status["age_s"], 2.1)
        self.assertEqual(status["source_age_s"], 0.4)
        self.assertEqual(status["state"], "tracking")

    def test_decode_depth_honors_step_padding(self):
        raw = np.array([[1000, 2000, 9999], [0, 500, 9999]], dtype="<u2")
        msg = SimpleNamespace(encoding="16UC1", is_bigendian=False,
                              step=6, width=2, height=2, data=raw.tobytes())
        depth = decode_depth(msg)
        np.testing.assert_allclose(depth, [[1.0, 2.0], [0.0, 0.5]])

    def test_depth_summary_ignores_zero_and_uses_center_window(self):
        depth = np.ones((20, 20), dtype=np.float32) * 2.0
        depth[0, 0] = 0.0
        depth[0:3, 0:3] = 0.75
        result = depth_summary(depth)
        self.assertEqual(result["width"], 20)
        self.assertGreater(result["valid_pct"], 99)
        self.assertEqual(result["center_m"], 2.0)
        self.assertLess(result["nearest_m"], 2.0)

    def test_operational_status_preserves_source_age_and_expires(self):
        now = [10.0]
        store = TelemetryStore(clock=lambda: now[0])
        store.update_navigation_json("execution", SimpleNamespace(data=json.dumps({
            "state": "BLOCKED", "blockers": ["perception_stale"], "age_s": .8,
        })), "/box_parking/execution_status")
        status = store.snapshot()["navigation"]["execution"]
        self.assertEqual(status["source_age_s"], .8)
        self.assertEqual(status["blockers"], ["perception_stale"])
        self.assertTrue(status["live"])
        now[0] = 12.1
        self.assertFalse(store.snapshot()["navigation"]["execution"]["live"])

    def test_malformed_status_invalidates_previous_ready(self):
        store = TelemetryStore()
        for text in ('{"state":"READY"}', '[]', '{"value":NaN}', 'not json'):
            store.update_navigation_json("execution", SimpleNamespace(data=text), "/status")
        status = store.snapshot()["navigation"]["execution"]
        self.assertFalse(status["live"])
        self.assertNotIn("state", status)
        self.assertTrue(status["error"])

    def test_old_source_cannot_be_reported_as_fresh_ready(self):
        store = TelemetryStore()
        for source_age in (9999, -1, "0", True):
            store.update_navigation_json("execution", SimpleNamespace(data=json.dumps({
                "state": "READY", "blockers": [], "age_s": source_age,
            })), "/execution")
            self.assertFalse(store.snapshot()["navigation"]["execution"]["live"])
        store.update_navigation_json("execution", SimpleNamespace(data=json.dumps({
            "state": "READY", "blockers": [], "age_s": .1,
        })), "/execution")
        self.assertTrue(store.snapshot()["navigation"]["execution"]["live"])

    def test_map_callback_only_retains_latest_and_rejects_oversized(self):
        store = TelemetryStore()
        first, last = self._map_message(), self._map_message(data=[0, 0, 0, 0])
        store.enqueue_map(first)
        store.enqueue_map(last)
        self.assertIs(store.latest_map[0], last)
        self.assertIsNone(store.map_png)
        store.render_map_pending()
        self.assertTrue(store.snapshot()["navigation"]["map"]["available"])
        self.assertIsNone(store.latest_map)
        store.enqueue_map(self._map_message(width=5000, height=5000))
        self.assertFalse(store.snapshot()["navigation"]["map"]["available"])
        self.assertIsNone(store.map_png)

    def test_system_snapshot_is_cached_not_resampled_per_http(self):
        now = [10.0]
        store = TelemetryStore(clock=lambda: now[0])
        store.update_system({"cpu_pct": 23., "cpu_temp_c": 82.})
        first = store.snapshot()
        first["system"]["cpu_pct"] = 999
        self.assertEqual(store.snapshot()["system"]["cpu_pct"], 23.)
        now[0] += 6.1
        self.assertFalse(store.snapshot()["system"]["live"])

    def test_graph_lists_names_and_duplicates_without_confusing_process_count(self):
        node = SimpleNamespace(
            get_node_names_and_namespaces=lambda: [("camera", "/"), ("camera", "/")],
            get_topic_names_and_types=lambda: [("/scan", ["sensor_msgs/msg/LaserScan"])])
        store = TelemetryStore()
        store.update_graph(node)
        graph = store.snapshot()["graph"]
        self.assertEqual(graph["nodes"], 2)
        self.assertEqual(graph["duplicates"], ["/camera"])
        self.assertEqual(graph["topic_names"], ["/scan"])

    def test_no_sensor_or_execution_data_does_not_claim_ready(self):
        state = TelemetryStore().snapshot()
        self.assertFalse(state["depth"]["live"])
        self.assertFalse(state["lidar"]["live"])
        self.assertFalse(state["navigation"]["map"]["available"])
        self.assertEqual(state["navigation"]["execution"], {})

    def test_latest_image_queue_is_bounded_and_does_not_encode_in_callback(self):
        now = [10.0]
        store = TelemetryStore(clock=lambda: now[0])
        for index in range(100):
            store.enqueue_image("depth", index, "/depth")
        self.assertEqual(store.latest_images["depth"], (99, "/depth", 10.0))
        self.assertIsNone(store.frames["depth"])
        self.assertLessEqual(len(store.times["depth"]), 90)

    @staticmethod
    def _map_message(width=2, height=2, data=(-1, 0, 100, 50)):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id="map"), data=data,
            info=SimpleNamespace(width=width, height=height, resolution=.05,
                                 origin=SimpleNamespace(
                                     position=SimpleNamespace(x=-1., y=-2.),
                                     orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))))

    def test_map_preview_preserves_origin_scale_and_flips_y(self):
        import cv2
        png, metadata = occupancy_preview(self._map_message())
        image = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        self.assertEqual(metadata["origin"], {"x_m": -1., "y_m": -2., "yaw_deg": 0.})
        self.assertEqual(metadata["known_pct"], 75.)
        self.assertEqual(int(image[0, 0]), 35)  # +y row occupied
        self.assertEqual(int(image[1, 0]), 213)  # -y row unknown
        self.assertEqual(int(image[1, 1]), 250)  # free cell

    def test_map_invalid_size_rejected_and_preview_bounded(self):
        with self.assertRaises(ValueError):
            occupancy_preview(self._map_message(width=0))
        with self.assertRaises(ValueError):
            occupancy_preview(self._map_message(data=[]))
        _, metadata = occupancy_preview(self._map_message(
            width=1024, height=2, data=[0] * 2048))
        self.assertEqual(metadata["pixel_width"], 512)

    def test_map_available_survives_static_map_update_gap_without_fake_liveness(self):
        now = [10.0]
        store = TelemetryStore(clock=lambda: now[0])
        store.update_map(self._map_message())
        now[0] += 30
        state = store.snapshot()["navigation"]["map"]
        self.assertTrue(state["available"])
        self.assertFalse(state["live"])

    def test_path_points_bounded_and_frame_not_silently_changed(self):
        pose = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1., y=2.)))
        store = TelemetryStore()
        store.update_path(SimpleNamespace(header=SimpleNamespace(frame_id="odom"),
                                          poses=[pose] * 2000))
        path = store.snapshot()["navigation"]["path"]
        self.assertLessEqual(len(path["points_xy_m"]), 400)
        self.assertEqual(path["frame_id"], "odom")

    def test_old_tf_cannot_keep_localization_alive(self):
        now = [10.0]
        store = TelemetryStore(clock=lambda: now[0])
        tf = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)),
            child_frame_id="base_footprint", transform=SimpleNamespace(
                translation=SimpleNamespace(x=1., y=2.),
                rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
        store.update_localization(tf, 100.1)
        self.assertTrue(store.snapshot()["navigation"]["localization"]["live"])
        now[0] += 3
        store.update_localization(tf, 103.1)
        self.assertFalse(store.snapshot()["navigation"]["localization"]["live"])

    def test_http_state_is_read_only_and_missing_map_is_explicit(self):
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(TelemetryStore()))
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"
        try:
            with urlopen(url + "/api/state", timeout=2) as response:
                self.assertFalse(json.load(response)["navigation"]["map"]["available"])
            with self.assertRaises(HTTPError) as error:
                urlopen(url + "/api/map.png", timeout=2)
            self.assertEqual(error.exception.code, 404)
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(url + "/api/drive", data=b'{}'), timeout=2)
            self.assertEqual(error.exception.code, 501)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
