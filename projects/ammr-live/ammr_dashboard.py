#!/usr/bin/env python3
"""AMMR 실기체 센서 관제 대시보드.

ROS 2 토픽을 구독만 하며 publisher·service client·로봇 명령 경로를 만들지 않는다.
HTTP에서는 최신 상태 JSON과 RGB/Depth MJPEG만 제공한다.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import pathlib
import signal
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import cv2
import numpy as np

from host_metrics import HostSampler


HERE = pathlib.Path(__file__).resolve().parent
STALE_SECONDS = 2.0
RENDER_PERIOD_SECONDS = 1.0 / 3.0


def yaw_degrees(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z)))


def message_age(msg):
    """Wall-clock age for live Pi sensor headers; absent stamps stay unknown."""
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    timestamp = stamp.sec + stamp.nanosec * 1e-9
    return round(time.time() - timestamp, 3) if timestamp > 0 else None


def occupancy_preview(msg):
    """Bound map image size; preserve metric origin and original cell dimensions."""
    width, height = int(msg.info.width), int(msg.info.height)
    resolution = finite(msg.info.resolution)
    if (width <= 0 or height <= 0 or width * height > 4_000_000
            or len(msg.data) != width * height
            or resolution is None or resolution <= 0):
        raise ValueError("invalid_occupancy_grid")
    cells = np.asarray(msg.data, dtype=np.int8).reshape(height, width)
    known = (cells >= 0) & (cells <= 100)
    pixels = np.full(cells.shape, 213, dtype=np.uint8)
    pixels[known] = (250 - cells[known] * 2.15).astype(np.uint8)
    scale = min(1.0, 512 / max(width, height))
    pixels = cv2.resize(np.flipud(pixels),
                        (max(1, round(width * scale)), max(1, round(height * scale))),
                        interpolation=cv2.INTER_NEAREST)
    ok, encoded = cv2.imencode('.png', pixels)
    if not ok:
        raise ValueError("map_encode_failed")
    origin = msg.info.origin
    values = (origin.position.x, origin.position.y, yaw_degrees(origin.orientation))
    if any(finite(value) is None for value in values):
        raise ValueError("invalid_map_origin")
    return encoded.tobytes(), {
        "available": True, "frame_id": msg.header.frame_id,
        "width": width, "height": height, "resolution_m": resolution,
        "known_pct": round(float(known.mean()) * 100, 1),
        "pixel_width": pixels.shape[1], "pixel_height": pixels.shape[0],
        "origin": dict(zip(("x_m", "y_m", "yaw_deg"), values)),
    }


def finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def finite_argument(value):
    number = finite(value)
    if number is None:
        raise argparse.ArgumentTypeError("유한한 숫자가 필요합니다")
    return number


def rolling_fps(timestamps):
    if len(timestamps) < 2:
        return 0.0
    span = timestamps[-1] - timestamps[0]
    return 0.0 if span <= 0 else (len(timestamps) - 1) / span


def scan_summary(ranges, angle_min, angle_increment, range_min, range_max,
                 max_points=240, yaw_deg=0.0, x_m=0.0, y_m=0.0):
    """LaserScan을 표시 프레임 XY로 변환해 방향별로 요약한다."""
    transform = [finite(value) for value in (yaw_deg, x_m, y_m)]
    if any(value is None for value in transform):
        raise ValueError("스캔 변환값은 유한한 숫자여야 합니다")
    yaw, offset_x, offset_y = math.radians(transform[0]), transform[1], transform[2]
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    valid = []
    for index, raw in enumerate(ranges):
        distance = finite(raw)
        if distance is None or distance < range_min or distance > range_max:
            continue
        angle = angle_min + index * angle_increment
        scan_x, scan_y = distance * math.cos(angle), distance * math.sin(angle)
        x = cos_yaw * scan_x - sin_yaw * scan_y + offset_x
        y = sin_yaw * scan_x + cos_yaw * scan_y + offset_y
        valid.append((math.atan2(y, x), math.hypot(x, y), x, y))

    def sector(center_deg, half_deg):
        center = math.radians(center_deg)
        half = math.radians(half_deg)
        values = [distance for angle, distance, _x, _y in valid
                  if abs(math.atan2(math.sin(angle - center),
                                    math.cos(angle - center))) <= half]
        return None if not values else round(min(values), 3)

    stride = max(1, math.ceil(len(valid) / max_points))
    points = [[round(x, 3), round(y, 3)]
              for _angle, _distance, x, y in valid[::stride]][:max_points]
    distances = [distance for _angle, distance, _x, _y in valid]
    return {
        "valid_points": len(valid),
        "min_m": None if not distances else round(min(distances), 3),
        "front_m": sector(0, 18),
        "left_m": sector(90, 24),
        "right_m": sector(-90, 24),
        "points_xy_m": points,
    }


def decode_depth(msg):
    """sensor_msgs/Image의 16UC1/mono16/32FC1을 미터 배열로 변환한다."""
    encoding = str(msg.encoding).lower()
    if encoding in ("16uc1", "mono16"):
        dtype, scale = np.dtype("u2"), 0.001
    elif encoding == "32fc1":
        dtype, scale = np.dtype("f4"), 1.0
    else:
        raise ValueError(f"지원하지 않는 depth encoding: {msg.encoding}")
    dtype = dtype.newbyteorder(">" if msg.is_bigendian else "<")
    row_values = int(msg.step) // dtype.itemsize
    array = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_values)
    return array[:, :msg.width].astype(np.float32) * scale


def depth_visual(depth_m):
    """0.2~5.0m 깊이를 가까운 곳=주황, 먼 곳=남색인 JPEG로 만든다."""
    valid = np.isfinite(depth_m) & (depth_m > 0.05)
    clipped = np.clip(depth_m, 0.2, 5.0)
    normalized = ((clipped - 0.2) / 4.8 * 255).astype(np.uint8)
    image = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
    image[~valid] = (21, 25, 35)
    return image


def depth_summary(depth_m):
    valid = np.isfinite(depth_m) & (depth_m > 0.05)
    values = depth_m[valid]
    height, width = depth_m.shape
    radius = max(2, min(height, width) // 40)
    center = depth_m[height // 2 - radius:height // 2 + radius + 1,
                     width // 2 - radius:width // 2 + radius + 1]
    center = center[np.isfinite(center) & (center > 0.05)]
    return {
        "width": width,
        "height": height,
        "valid_pct": round(float(valid.mean() * 100), 1),
        "center_m": None if not center.size else round(float(np.median(center)), 3),
        "nearest_m": None if not values.size else round(float(np.percentile(values, 1)), 3),
    }


def image_to_bgr(msg):
    encoding = str(msg.encoding).lower()
    channels = 1 if encoding in ("mono8", "8uc1") else 3
    row_values = int(msg.step)
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, row_values)
    if channels == 1:
        return cv2.cvtColor(raw[:, :msg.width], cv2.COLOR_GRAY2BGR)
    image = raw[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding in ("bgr8", "8uc3"):
        return image
    raise ValueError(f"지원하지 않는 color encoding: {msg.encoding}")


class TelemetryStore:
    def __init__(self, clock=time.monotonic, scan_yaw_deg=0.0, scan_x_m=0.0,
                 scan_y_m=0.0, scan_display_frame="laser_link"):
        self.clock = clock
        self.lock = threading.Lock()
        self.frame_ready = {"depth": threading.Condition(self.lock),
                            "color": threading.Condition(self.lock)}
        self.frames = {"depth": None, "color": None}
        self.latest_images = {"depth": None, "color": None}
        self.latest_map = None
        self.map_png = None
        self.map_version = 0
        self.sequences = {"depth": 0, "color": 0}
        self.last_render_at = {"depth": 0.0, "color": 0.0}
        self.times = {"depth": deque(maxlen=90), "color": deque(maxlen=90),
                      "scan": deque(maxlen=90)}
        self.scan_transform = {
            "yaw_deg": scan_yaw_deg, "x_m": scan_x_m, "y_m": scan_y_m,
            "display_frame": scan_display_frame,
        }
        self.state = {
            "started_at": self.clock(),
            "depth": {"topic": None, "error": None},
            "color": {"topic": None, "error": None},
            "lidar": {"topic": "/scan", **self.scan_transform},
            "depth_navigation": {"topic": "/depth_navigation/status"},
            "motion": {"odom": None, "cmd": None, "requested_cmd": None},
            "battery": None,
            "imu": None,
            "graph": {"nodes": 0, "topics": 0},
            "system": {},
            "navigation": {"execution": {}, "perception": {}, "collision": {},
                           "goal": {}, "map": {"available": False},
                           "localization": {}, "path": {}},
        }

    def _stamp(self, key):
        now = self.clock()
        self.times[key].append(now)
        return now

    def enqueue_image(self, kind, msg, topic):
        """ROS callbacks retain only the latest frame; rendering runs elsewhere."""
        with self.lock:
            now = self._stamp(kind)
            self.latest_images[kind] = (msg, topic, now)

    def render_pending(self):
        with self.lock:
            pending = self.latest_images
            self.latest_images = {"depth": None, "color": None}
        for kind, item in pending.items():
            if item is not None:
                getattr(self, f"update_{kind}")(*item)

    def enqueue_map(self, msg):
        """ROS retains one bounded grid; rasterization never runs in its callback."""
        cells = int(msg.info.width) * int(msg.info.height)
        if cells <= 0 or cells > 4_000_000 or len(msg.data) != cells:
            with self.lock:
                self.latest_map = None
                self.map_png = None
                self.state["navigation"]["map"] = {
                    "available": False, "error": "invalid_or_oversized_map",
                    "observed_at": self.clock()}
            return
        with self.lock:
            self.latest_map = (msg, self.clock())

    def render_map_pending(self):
        with self.lock:
            pending, self.latest_map = self.latest_map, None
        if pending is not None:
            self.update_map(*pending)

    def update_depth(self, msg, topic, received_at=None):
        now = self._stamp("depth") if received_at is None else received_at
        render_at = self.clock()
        if render_at - self.last_render_at["depth"] < RENDER_PERIOD_SECONDS:
            return
        self.last_render_at["depth"] = render_at
        try:
            depth = decode_depth(msg)
            summary = depth_summary(depth)
            image = depth_visual(depth)
            cv2.drawMarker(image, (image.shape[1] // 2, image.shape[0] // 2),
                           (255, 255, 255), cv2.MARKER_CROSS, 18, 1)
            ok, encoded = cv2.imencode(".jpg", image,
                                      [cv2.IMWRITE_JPEG_QUALITY, 82])
            if not ok:
                raise RuntimeError("JPEG 인코딩 실패")
            with self.lock:
                self.frames["depth"] = encoded.tobytes()
                self.sequences["depth"] += 1
                self.state["depth"] = {
                    **summary, "topic": topic, "encoding": msg.encoding,
                    "fps": round(rolling_fps(self.times["depth"]), 1),
                    "observed_at": now, "source_age_s": message_age(msg),
                    "preview_fps_limit": 3, "error": None,
                }
                self.frame_ready["depth"].notify_all()
        except Exception as exc:
            with self.lock:
                self.state["depth"]["error"] = f"{type(exc).__name__}: {exc}"

    def update_color(self, msg, topic, received_at=None):
        now = self._stamp("color") if received_at is None else received_at
        render_at = self.clock()
        if render_at - self.last_render_at["color"] < RENDER_PERIOD_SECONDS:
            return
        self.last_render_at["color"] = render_at
        try:
            image = image_to_bgr(msg)
            ok, encoded = cv2.imencode(".jpg", image,
                                      [cv2.IMWRITE_JPEG_QUALITY, 78])
            if not ok:
                raise RuntimeError("JPEG 인코딩 실패")
            with self.lock:
                self.frames["color"] = encoded.tobytes()
                self.sequences["color"] += 1
                self.state["color"] = {
                    "topic": topic, "encoding": msg.encoding,
                    "width": msg.width, "height": msg.height,
                    "fps": round(rolling_fps(self.times["color"]), 1),
                    "observed_at": now, "source_age_s": message_age(msg),
                    "preview_fps_limit": 3, "error": None,
                }
                self.frame_ready["color"].notify_all()
        except Exception as exc:
            with self.lock:
                self.state["color"]["error"] = f"{type(exc).__name__}: {exc}"

    def update_scan(self, msg):
        now = self._stamp("scan")
        summary = scan_summary(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max,
                               yaw_deg=self.scan_transform["yaw_deg"],
                               x_m=self.scan_transform["x_m"],
                               y_m=self.scan_transform["y_m"])
        with self.lock:
            self.state["lidar"] = {
                **summary, "topic": "/scan", "observed_at": now,
                "fps": round(rolling_fps(self.times["scan"]), 1),
                "source_age_s": message_age(msg),
                **self.scan_transform,
            }

    def update_depth_navigation(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("JSON object가 아님")
            json.dumps(payload, allow_nan=False)
            source_age = payload.pop("age_s", None)
            state = {**payload, "topic": "/depth_navigation/status",
                     "observed_at": self.clock(), "error": None}
            if source_age is not None:
                state["source_age_s"] = source_age
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            state = {"topic": "/depth_navigation/status",
                     "observed_at": self.clock(),
                     "error": f"{type(exc).__name__}: {exc}"}
        with self.lock:
            self.state["depth_navigation"] = state

    def update_motion(self, key, linear, angular, **extra):
        with self.lock:
            self.state["motion"][key] = {
                "linear_mps": finite(linear),
                "angular_rps": finite(angular),
                "observed_at": self.clock(), **extra,
            }

    def update_battery(self, msg):
        percentage = finite(msg.percentage)
        if percentage is not None and 0 <= percentage <= 1:
            percentage *= 100
        else:
            percentage = None
        with self.lock:
            self.state["battery"] = {
                "voltage_v": finite(msg.voltage),
                "percentage_pct": percentage,
                "present": bool(msg.present), "observed_at": self.clock(),
            }

    def update_imu(self, msg):
        with self.lock:
            self.state["imu"] = {
                "angular_z_rps": finite(msg.angular_velocity.z),
                "accel_x_mps2": finite(msg.linear_acceleration.x),
                "accel_y_mps2": finite(msg.linear_acceleration.y),
                "observed_at": self.clock(),
            }

    def update_graph(self, node):
        # DDS lookups must not hold the HTTP/frame lock.
        names = sorted(f"{namespace.rstrip('/')}/{name}"
                       for name, namespace in node.get_node_names_and_namespaces())
        topics = sorted(name for name, _types in node.get_topic_names_and_types())
        with self.lock:
            self.state["graph"] = {
                "nodes": len(names), "topics": len(topics),
                "node_names": names, "topic_names": topics,
                "duplicates": sorted({name for name in names if names.count(name) > 1}),
                "observed_at": self.clock(),
            }

    def update_system(self, sample):
        with self.lock:
            self.state["system"] = {**sample, "observed_at": self.clock()}

    def update_navigation_json(self, key, msg, topic):
        try:
            if len(msg.data) > 32768:
                raise ValueError("status_payload_too_large")
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("status_not_object")
            json.dumps(payload, allow_nan=False)
            source_age = payload.pop("age_s", None)
            if source_age is not None and (finite(source_age) is None
                                           or isinstance(source_age, (str, bool))
                                           or source_age < 0):
                raise ValueError("invalid_source_age")
            if key == "execution" and (payload.get("state") not in {
                    "READY", "BLOCKED", "ARMED", "ABORTED", "SUCCEEDED"}
                    or not isinstance(payload.get("blockers"), list)
                    or not all(isinstance(x, str) for x in payload["blockers"])):
                raise ValueError("invalid_execution_schema")
            if key == "perception" and (not isinstance(payload.get("detected"), bool)
                                        or not isinstance(payload.get("stable"), bool)):
                raise ValueError("invalid_perception_schema")
            if "stamp_s" in payload:
                stamp = finite(payload["stamp_s"])
                if stamp is None or stamp <= 0:
                    raise ValueError("invalid_source_stamp")
                source_age = time.time() - stamp
            payload = {**payload, "source_age_s": source_age, "error": None}
        except (TypeError, ValueError) as exc:
            payload = {"error": str(exc)}
        with self.lock:
            self.state["navigation"][key] = {
                **payload, "topic": topic, "observed_at": self.clock()}

    def update_collision(self, msg):
        actions = {0: "DO_NOTHING", 1: "STOP", 2: "SLOWDOWN", 3: "APPROACH", 4: "LIMIT"}
        with self.lock:
            self.state["navigation"]["collision"] = {
                "action": actions.get(msg.action_type, "UNKNOWN"),
                "polygon": msg.polygon_name, "observed_at": self.clock(),
            }

    def update_goal(self, msg):
        if not msg.status_list:
            return
        status = max(msg.status_list, key=lambda item:
                     item.goal_info.stamp.sec + item.goal_info.stamp.nanosec * 1e-9)
        labels = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
                  4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        with self.lock:
            self.state["navigation"]["goal"] = {
                "status": labels.get(status.status, "UNKNOWN"),
                "goal_id": bytes(status.goal_info.goal_id.uuid).hex(),
                "observed_at": self.clock()}

    def update_map(self, msg, received_at=None):
        now = self.clock() if received_at is None else received_at
        try:
            png, metadata = occupancy_preview(msg)
        except (ValueError, TypeError, cv2.error) as exc:
            with self.lock:
                self.state["navigation"]["map"] = {
                    "available": False, "error": str(exc), "observed_at": now}
                self.map_png = None
            return
        with self.lock:
            self.map_version += 1
            self.map_png = png
            self.state["navigation"]["map"] = {
                **metadata, "version": self.map_version,
                "observed_at": now, "error": None}

    def update_path(self, msg):
        stride = max(1, math.ceil(len(msg.poses) / 400))
        points = [[finite(p.pose.position.x), finite(p.pose.position.y)]
                  for p in msg.poses[::stride]]
        with self.lock:
            self.state["navigation"]["path"] = {
                "points_xy_m": [p for p in points if None not in p],
                "frame_id": msg.header.frame_id, "observed_at": self.clock()}

    def update_localization(self, transform, now_ros_s):
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        pose = transform.transform
        values = [finite(pose.translation.x), finite(pose.translation.y),
                  finite(yaw_degrees(pose.rotation))]
        if None in values or stamp <= 0 or not 0 <= now_ros_s - stamp <= 2:
            return
        with self.lock:
            self.state["navigation"]["localization"] = {
                "x_m": values[0], "y_m": values[1], "yaw_deg": values[2],
                "frame_id": "map", "child_frame_id": transform.child_frame_id,
                "source_age_s": now_ros_s - stamp,
                "observed_at": self.clock()}

    def snapshot(self):
        now = self.clock()
        with self.lock:
            result = copy.deepcopy(self.state)
        result["server"] = {
            "uptime_s": round(now - result.pop("started_at"), 1),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "discovery_range": os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        }
        def freshness(item, limit=STALE_SECONDS):
            if item:
                observed = item.pop("observed_at", None)
                item["age_s"] = None if observed is None else round(now - observed, 2)
                item["live"] = (observed is not None and 0 <= now - observed < limit
                                and not item.get("error"))
        for group in ("depth", "color", "lidar", "battery", "imu", "depth_navigation"):
            freshness(result.get(group))
        freshness(result["graph"], 12.0)
        freshness(result["system"], 6.0)
        for item in result["motion"].values():
            freshness(item)
        for item in result["navigation"].values():
            freshness(item)
        # Arrival freshness must not turn an already-old payload into fresh data.
        for item in [result[group] for group in ("depth", "color", "lidar",
                                                "depth_navigation")] + list(
                result["navigation"].values()):
            source_age = item.get("source_age_s")
            if source_age is not None and item.get("live"):
                value = finite(source_age)
                item["live"] = (value is not None and not isinstance(source_age, (str, bool))
                                and 0 <= value + item["age_s"] < STALE_SECONDS)
        return result

    def wait_frame(self, kind, after, timeout=2.0):
        deadline = self.clock() + timeout
        with self.frame_ready[kind]:
            while self.sequences[kind] <= after:
                remaining = deadline - self.clock()
                if remaining <= 0:
                    return self.sequences[kind], self.frames[kind]
                self.frame_ready[kind].wait(remaining)
            return self.sequences[kind], self.frames[kind]


def make_handler(store):
    class Handler(BaseHTTPRequestHandler):
        server_version = "AMMRDashboard/1.0"

        def log_message(self, fmt, *args):
            return

        def _headers(self, code, content_type, length=None):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            if length is not None:
                self.send_header("Content-Length", str(length))
            self.end_headers()

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                body = (HERE / "dashboard.html").read_bytes()
                self._headers(200, "text/html; charset=utf-8", len(body))
                self.wfile.write(body)
            elif path == "/dashboard.js":
                body = (HERE / "dashboard.js").read_bytes()
                self._headers(200, "text/javascript; charset=utf-8", len(body))
                self.wfile.write(body)
            elif path == "/api/map.png":
                with store.lock:
                    body = store.map_png
                if body is None:
                    self._headers(404, "text/plain; charset=utf-8", 0)
                else:
                    self._headers(200, "image/png", len(body))
                    self.wfile.write(body)
            elif path == "/api/state":
                body = json.dumps(store.snapshot(), ensure_ascii=False,
                                  allow_nan=False).encode()
                self._headers(200, "application/json; charset=utf-8", len(body))
                self.wfile.write(body)
            elif path == "/health":
                state = store.snapshot()
                ok = state["depth"].get("live") and state["lidar"].get("live")
                body = json.dumps({"ok": bool(ok), "depth": state["depth"],
                                   "lidar": state["lidar"]},
                                  ensure_ascii=False).encode()
                self._headers(200 if ok else 503, "application/json", len(body))
                self.wfile.write(body)
            elif path in ("/stream/depth.mjpg", "/stream/color.mjpg"):
                self._stream(path.split("/")[-1].split(".")[0])
            else:
                self._headers(404, "text/plain; charset=utf-8", 9)
                self.wfile.write("not found".encode())

        def _stream(self, kind):
            self.connection.settimeout(5.0)
            self._headers(200, "multipart/x-mixed-replace; boundary=frame")
            sequence = -1
            try:
                while True:
                    next_sequence, jpeg = store.wait_frame(kind, sequence)
                    if jpeg is None or next_sequence <= sequence:
                        continue
                    sequence = next_sequence
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                return

    return Handler


def build_node(store, depth_topic, color_topic):
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import BatteryState, Image, Imu, LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)

    class DashboardNode(Node):
        def __init__(self):
            super().__init__("ammr_dashboard_monitor")
            self.create_subscription(Image, depth_topic,
                                     lambda msg: store.enqueue_image("depth", msg, depth_topic),
                                     sensor_qos)
            self.create_subscription(Image, color_topic,
                                     lambda msg: store.enqueue_image("color", msg, color_topic),
                                     sensor_qos)
            self.create_subscription(LaserScan, "/scan", store.update_scan,
                                     sensor_qos)
            self.create_subscription(Odometry, "/odom", self.odom,
                                     sensor_qos)
            self.create_subscription(Twist, "/cmd_vel", self.cmd,
                                     sensor_qos)
            self.create_subscription(Twist, "/cmd_vel_smoothed", self.requested,
                                     sensor_qos)
            self.create_subscription(Twist, "/cmd_vel_nav",
                                     lambda msg: store.update_motion(
                                         "input_cmd", msg.linear.x, msg.angular.z), sensor_qos)
            self.create_subscription(BatteryState, "/battery_state",
                                     store.update_battery, sensor_qos)
            self.create_subscription(Imu, "/imu/data_raw", store.update_imu,
                                     sensor_qos)
            self.create_subscription(String, "/depth_navigation/status",
                                     store.update_depth_navigation, 10)
            for key, topic in (("execution", "/box_parking/execution_status"),
                               ("perception", "/box_parking/perception_status")):
                self.create_subscription(String, topic,
                                         lambda msg, k=key, t=topic:
                                         store.update_navigation_json(k, msg, t), 1)
            self.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                     store.update_collision, sensor_qos)
            self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                     store.update_goal, sensor_qos)
            self.create_subscription(OccupancyGrid, "/map", store.enqueue_map, map_qos)
            self.create_subscription(Path, "/plan", store.update_path, sensor_qos)
            self.tf_buffer = Buffer(cache_time=Duration(seconds=5))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_timer(1.0, self.localization)
            self.create_timer(5.0, lambda: store.update_graph(self))

        def localization(self):
            for target in ("base_footprint", "base_link"):
                try:
                    transform = self.tf_buffer.lookup_transform("map", target, Time())
                    store.update_localization(transform,
                                              self.get_clock().now().nanoseconds * 1e-9)
                    return
                except TransformException:
                    continue

        def odom(self, msg):
            pose = msg.pose.pose
            q = pose.orientation
            yaw = yaw_degrees(q)
            store.update_motion("odom", msg.twist.twist.linear.x,
                                msg.twist.twist.angular.z,
                                x_m=finite(pose.position.x),
                                y_m=finite(pose.position.y), yaw_deg=finite(yaw))

        def cmd(self, msg):
            store.update_motion("cmd", msg.linear.x, msg.angular.z)

        def requested(self, msg):
            store.update_motion("requested_cmd", msg.linear.x, msg.angular.z)

    if not rclpy.ok():
        rclpy.init()
    return DashboardNode()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--scan-yaw-deg", type=finite_argument, default=0.0)
    parser.add_argument("--scan-x-m", type=finite_argument, default=0.0)
    parser.add_argument("--scan-y-m", type=finite_argument, default=0.0)
    parser.add_argument("--scan-display-frame", default="laser_link")
    args = parser.parse_args()

    import rclpy
    # The preview must not create a competing OpenCV worker pool on the Pi.
    cv2.setNumThreads(1)
    store = TelemetryStore(scan_yaw_deg=args.scan_yaw_deg,
                           scan_x_m=args.scan_x_m,
                           scan_y_m=args.scan_y_m,
                           scan_display_frame=args.scan_display_frame)
    stopping = threading.Event()
    sampler = HostSampler()

    def sample_host():
        while not stopping.is_set():
            try:
                store.update_system(sampler.sample())
            except Exception as exc:
                store.update_system({"error": str(exc), "errors": [str(exc)]})
            stopping.wait(2.0)

    def render_images():
        while not stopping.is_set():
            store.render_pending()
            stopping.wait(RENDER_PERIOD_SECONDS)

    def render_maps():
        while not stopping.is_set():
            store.render_map_pending()
            stopping.wait(2.0)

    node = build_node(store, args.depth_topic, args.color_topic)
    server = ThreadingHTTPServer((args.bind, args.port), make_handler(store))
    server.daemon_threads = True
    server.block_on_close = False
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    threads = [threading.Thread(target=sample_host, daemon=True),
               threading.Thread(target=render_images, daemon=True),
               threading.Thread(target=render_maps, daemon=True)]
    for thread in threads:
        thread.start()
    print(f"AMMR 대시보드: http://{args.bind}:{args.port}", flush=True)

    def stop(_signum=None, _frame=None):
        stopping.set()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    finally:
        stopping.set()
        server.shutdown()
        server.server_close()
        node.destroy_node()
        for thread in threads:
            thread.join(timeout=2.0)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
