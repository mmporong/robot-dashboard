#!/usr/bin/env python3
"""Read-only host metrics for the AMMR dashboard.

CPU percentages use the whole machine as 100%, rather than reporting 100% per
logical core.  The sampler owns no thread or timer; callers decide when to call
``sample``.
"""

from __future__ import annotations

import os
import pathlib
import shutil
import socket
import subprocess
import time


SERVICE_ALLOWLIST = (
    "jdamr-base.service",
    "jdamr-box-rgbd.service",
    "jdamr-box-approach.service",
    "jdamr-ammr-dashboard.service",
)

THROTTLE_FLAGS = (
    (0, "저전압 감지"),
    (1, "주파수 제한"),
    (2, "스로틀링"),
    (3, "소프트 온도 제한"),
)


class HostSampler:
    """Collect a bounded snapshot of Linux and Raspberry Pi host state."""

    def __init__(self, clock=time.monotonic, proc_root="/proc", sys_root="/sys",
                 disk_path="/", runner=subprocess.run,
                 disk_usage=shutil.disk_usage, hostname_getter=socket.gethostname,
                 cpu_count_getter=os.cpu_count, page_size=None,
                 process_interval_s=5.0, command_interval_s=10.0):
        self.clock = clock
        self.proc_root = pathlib.Path(proc_root)
        self.sys_root = pathlib.Path(sys_root)
        self.disk_path = disk_path
        self.runner = runner
        self.disk_usage = disk_usage
        self.hostname_getter = hostname_getter
        self.cpu_count_getter = cpu_count_getter
        self.page_size = page_size or os.sysconf("SC_PAGE_SIZE")
        self.process_interval_s = max(5.0, float(process_interval_s))
        self.command_interval_s = max(10.0, float(command_interval_s))
        self._cpu_previous = None
        self._process_previous = {}
        self._process_total_previous = None
        self._process_cache = []
        self._process_at = None
        self._process_error = None
        self._throttle_cache = self._unknown_throttle()
        self._throttle_at = None
        self._throttle_error = None
        self._services_cache = [
            {"name": name, "state": "unknown"} for name in SERVICE_ALLOWLIST
        ]
        self._services_at = None
        self._services_error = None

    @staticmethod
    def _unknown_throttle():
        return {"available": False, "raw": None, "current": [], "historical": []}

    @staticmethod
    def _error(errors, label, exc):
        errors.append(f"{label}: {type(exc).__name__}: {exc}")

    def _read_text(self, path):
        return pathlib.Path(path).read_text(encoding="utf-8", errors="replace")

    def _cpu_times(self):
        line = self._read_text(self.proc_root / "stat").splitlines()[0]
        parts = line.split()
        if not parts or parts[0] != "cpu":
            raise ValueError("/proc/stat의 cpu 행이 없습니다")
        values = [int(value) for value in parts[1:]]
        if len(values) < 4:
            raise ValueError("/proc/stat의 cpu 필드가 부족합니다")
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        # guest and guest_nice are already included in user and nice.
        return sum(values[:8]), idle

    def _cpu_percent(self, current):
        previous, self._cpu_previous = self._cpu_previous, current
        if previous is None:
            return None
        total_delta = current[0] - previous[0]
        idle_delta = current[1] - previous[1]
        if total_delta <= 0 or idle_delta < 0:
            return None
        return round(max(0.0, min(100.0, (total_delta - idle_delta) * 100.0 /
                                        total_delta)), 1)

    def _load_averages(self):
        values = self._read_text(self.proc_root / "loadavg").split()
        if len(values) < 3:
            raise ValueError("/proc/loadavg 필드가 부족합니다")
        return tuple(float(value) for value in values[:3])

    def _memory(self):
        fields = {}
        for line in self._read_text(self.proc_root / "meminfo").splitlines():
            key, separator, value = line.partition(":")
            if separator:
                fields[key] = int(value.split()[0])
        total = fields["MemTotal"]
        available = fields.get("MemAvailable")
        if available is None:
            available = sum(fields.get(key, 0)
                            for key in ("MemFree", "Buffers", "Cached"))
        used = max(0, total - available)
        swap_total = fields.get("SwapTotal", 0)
        swap_used = max(0, swap_total - fields.get("SwapFree", 0))
        return {
            "memory_pct": None if total <= 0 else round(used * 100.0 / total, 1),
            "memory_used_mb": round(used * 1024.0 / 1_000_000, 1),
            "memory_total_mb": round(total * 1024.0 / 1_000_000, 1),
            "swap_pct": None if swap_total <= 0 else round(
                swap_used * 100.0 / swap_total, 1),
        }

    def _disk(self):
        usage = self.disk_usage(self.disk_path)
        used = usage.total - usage.free
        return {
            "disk_pct": None if usage.total <= 0 else round(
                used * 100.0 / usage.total, 1),
            "disk_free_gb": round(usage.free / 1_000_000_000, 1),
        }

    def _temperature(self):
        candidates = list((self.sys_root / "class/thermal").glob(
            "thermal_zone*/temp"))
        candidates.extend((self.sys_root / "class/hwmon").glob("hwmon*/temp1_input"))
        for path in candidates:
            try:
                value = float(self._read_text(path).strip())
                temperature = value / 1000.0 if abs(value) >= 1000 else value
                if -50 <= temperature <= 200:
                    return round(temperature, 1)
            except (OSError, ValueError):
                continue
        raise FileNotFoundError("사용 가능한 CPU 온도 센서가 없습니다")

    def _frequency(self):
        values = []
        for path in (self.sys_root / "devices/system/cpu").glob(
                "cpu[0-9]*/cpufreq/scaling_cur_freq"):
            try:
                values.append(float(self._read_text(path).strip()) / 1000.0)
            except (OSError, ValueError):
                continue
        if not values:
            for line in self._read_text(self.proc_root / "cpuinfo").splitlines():
                key, separator, value = line.partition(":")
                if separator and key.strip().lower() == "cpu mhz":
                    values.append(float(value.strip()))
        if not values:
            raise FileNotFoundError("CPU 주파수 정보를 찾을 수 없습니다")
        return round(sum(values) / len(values), 1)

    def _process_record(self, directory):
        raw = self._read_text(directory / "stat").strip()
        closing = raw.rfind(")")
        opening = raw.find("(")
        if opening < 0 or closing <= opening:
            raise ValueError("프로세스 stat 형식이 잘못됐습니다")
        pid = int(raw[:opening].strip())
        fields = raw[closing + 2:].split()  # starts at field 3 (state)
        ticks = int(fields[11]) + int(fields[12])
        starttime = int(fields[19])
        rss_pages = int(fields[21])
        try:
            name = self._read_text(directory / "comm").strip()
        except OSError:
            name = raw[opening + 1:closing]
        return pid, name, ticks, starttime, max(0, rss_pages)

    def _scan_processes(self, now, system_total, errors):
        if (self._process_at is not None and
                now - self._process_at < self.process_interval_s):
            if self._process_error:
                errors.append(self._process_error)
            return list(self._process_cache)
        self._process_at = now
        self._process_error = None
        try:
            records = []
            current = {}
            for directory in self.proc_root.iterdir():
                if not directory.name.isdigit() or not directory.is_dir():
                    continue
                try:
                    pid, name, ticks, starttime, rss_pages = self._process_record(directory)
                except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError,
                        IndexError):
                    continue
                current[pid] = (ticks, starttime)
                previous = self._process_previous.get(pid)
                cpu_pct = None
                if (previous is not None and previous[1] == starttime and
                        self._process_total_previous is not None):
                    total_delta = system_total - self._process_total_previous
                    tick_delta = ticks - previous[0]
                    if total_delta > 0 and tick_delta >= 0:
                        cpu_pct = round(min(100.0, tick_delta * 100.0 / total_delta), 1)
                records.append({
                    "pid": pid,
                    "name": name,
                    "cpu_pct": cpu_pct,
                    "memory_mb": round(rss_pages * self.page_size / 1_000_000, 1),
                })
            records.sort(key=lambda item: (
                item["cpu_pct"] is not None,
                -1.0 if item["cpu_pct"] is None else item["cpu_pct"],
                item["memory_mb"], -item["pid"]), reverse=True)
            self._process_previous = current
            self._process_total_previous = system_total
            self._process_cache = records[:6]
        except Exception as exc:  # A disappearing /proc must not break the server.
            self._error(errors, "top_processes", exc)
            self._process_error = errors[-1]
        return list(self._process_cache)

    def _run(self, args, timeout):
        return self.runner(args, capture_output=True, text=True, timeout=timeout,
                           check=False, shell=False)

    def _throttle(self, now, errors):
        if (self._throttle_at is not None and
                now - self._throttle_at < self.command_interval_s):
            if self._throttle_error:
                errors.append(self._throttle_error)
            return {**self._throttle_cache,
                    "current": list(self._throttle_cache["current"]),
                    "historical": list(self._throttle_cache["historical"])}
        result = self._unknown_throttle()
        self._throttle_error = None
        try:
            completed = self._run(["vcgencmd", "get_throttled"], timeout=0.5)
            if completed.returncode != 0:
                raise RuntimeError(completed.stderr.strip() or
                                   f"종료 코드 {completed.returncode}")
            output = completed.stdout.strip()
            prefix, separator, value = output.partition("=")
            if separator != "=" or prefix.strip() != "throttled":
                raise ValueError("vcgencmd 응답 형식이 잘못됐습니다")
            raw = int(value.strip(), 0)
            result = {
                "available": True,
                "raw": raw,
                "current": [label for bit, label in THROTTLE_FLAGS
                            if raw & (1 << bit)],
                "historical": [label for bit, label in THROTTLE_FLAGS
                               if raw & (1 << (bit + 16))],
            }
        except Exception as exc:
            self._error(errors, "throttle", exc)
            self._throttle_error = errors[-1]
        self._throttle_cache = result
        self._throttle_at = now
        return dict(result)

    def _services(self, now, errors):
        if (self._services_at is not None and
                now - self._services_at < self.command_interval_s):
            if self._services_error:
                errors.append(self._services_error)
            return [dict(item) for item in self._services_cache]
        services = [{"name": name, "state": "unknown"}
                    for name in SERVICE_ALLOWLIST]
        self._services_error = None
        try:
            completed = self._run([
                "systemctl", "show", "--property=Id,ActiveState,SubState",
                *SERVICE_ALLOWLIST,
            ], timeout=1.0)
            if completed.returncode != 0:
                # systemctl can return valid blocks along with a missing unit.
                self._error(errors, "services", RuntimeError(
                    completed.stderr.strip() or f"종료 코드 {completed.returncode}"))
                self._services_error = errors[-1]
            parsed = {}
            block = {}
            for line in completed.stdout.splitlines() + [""]:
                if not line.strip():
                    service_id = block.get("Id")
                    if service_id in SERVICE_ALLOWLIST:
                        active = block.get("ActiveState", "unknown")
                        sub = block.get("SubState")
                        parsed[service_id] = (f"{active} ({sub})" if sub and sub != active
                                              else active)
                    block = {}
                    continue
                key, separator, value = line.partition("=")
                if separator:
                    block[key] = value
            services = [{"name": name, "state": parsed.get(name, "unknown")}
                        for name in SERVICE_ALLOWLIST]
        except Exception as exc:
            self._error(errors, "services", exc)
            self._services_error = errors[-1]
        self._services_cache = services
        self._services_at = now
        return [dict(item) for item in services]

    def sample(self):
        """Return one best-effort snapshot; individual failures never escape."""
        try:
            now = self.clock()
        except Exception:
            now = time.monotonic()
        result = {
            "cpu_pct": None,
            "cpu_count": None,
            "load_1m": None,
            "load_5m": None,
            "load_15m": None,
            "memory_pct": None,
            "memory_used_mb": None,
            "memory_total_mb": None,
            "swap_pct": None,
            "disk_pct": None,
            "disk_free_gb": None,
            "cpu_temp_c": None,
            "cpu_freq_mhz": None,
            "uptime_s": None,
            "hostname": None,
            "sampled_at": now,
            "top_processes": [],
            "throttle": self._unknown_throttle(),
            "services": [{"name": name, "state": "unknown"}
                         for name in SERVICE_ALLOWLIST],
            "errors": [],
        }
        errors = result["errors"]
        cpu_times = None
        try:
            cpu_times = self._cpu_times()
            result["cpu_pct"] = self._cpu_percent(cpu_times)
        except Exception as exc:
            self._error(errors, "cpu", exc)
        try:
            count = self.cpu_count_getter()
            result["cpu_count"] = int(count) if count is not None else None
        except Exception as exc:
            self._error(errors, "cpu_count", exc)
        try:
            result["load_1m"], result["load_5m"], result["load_15m"] = (
                self._load_averages())
        except Exception as exc:
            self._error(errors, "load", exc)
        try:
            result.update(self._memory())
        except Exception as exc:
            self._error(errors, "memory", exc)
        try:
            result.update(self._disk())
        except Exception as exc:
            self._error(errors, "disk", exc)
        try:
            result["cpu_temp_c"] = self._temperature()
        except Exception as exc:
            self._error(errors, "cpu_temp_c", exc)
        try:
            result["cpu_freq_mhz"] = self._frequency()
        except Exception as exc:
            self._error(errors, "cpu_freq_mhz", exc)
        try:
            result["uptime_s"] = float(
                self._read_text(self.proc_root / "uptime").split()[0])
        except Exception as exc:
            self._error(errors, "uptime_s", exc)
        try:
            result["hostname"] = self.hostname_getter()
        except Exception as exc:
            self._error(errors, "hostname", exc)
        if cpu_times is not None:
            result["top_processes"] = self._scan_processes(now, cpu_times[0], errors)
        result["throttle"] = self._throttle(now, errors)
        result["services"] = self._services(now, errors)
        return result
