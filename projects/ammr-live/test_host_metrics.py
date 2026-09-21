import pathlib
import sys
import tempfile
import unittest
from collections import namedtuple
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from host_metrics import HostSampler, SERVICE_ALLOWLIST  # noqa: E402


DiskUsage = namedtuple("DiskUsage", "total used free")


class FakeRunner:
    def __init__(self, throttle="throttled=0x0\n", services="", missing=False):
        self.throttle = throttle
        self.services = services
        self.missing = missing
        self.calls = []

    def __call__(self, args, **kwargs):
        self.calls.append((args, kwargs))
        if self.missing:
            raise FileNotFoundError(args[0])
        stdout = self.throttle if args[0] == "vcgencmd" else self.services
        return SimpleNamespace(returncode=0, stdout=stdout, stderr="")


class HostSamplerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self.temp.name)
        self.proc = self.root / "proc"
        self.sys = self.root / "sys"
        self.proc.mkdir()
        self.sys.mkdir()
        self.now = [100.0]
        self.runner = FakeRunner(services="\n".join(
            f"Id={name}\nActiveState=active\nSubState=running\n"
            for name in SERVICE_ALLOWLIST))
        self._write_common()

    def tearDown(self):
        self.temp.cleanup()

    def _write(self, relative, content):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    def _write_common(self):
        self._write("proc/stat", "cpu  100 0 50 850 0 0 0 0\ncpu0 1 0 0 9\n")
        self._write("proc/loadavg", "0.10 0.20 0.30 1/100 123\n")
        self._write("proc/meminfo", "MemTotal: 1048576 kB\nMemAvailable: 524288 kB\n"
                    "SwapTotal: 262144 kB\nSwapFree: 196608 kB\n")
        self._write("proc/cpuinfo", "processor : 0\ncpu MHz : 1500.0\n")
        self._write("proc/uptime", "321.5 100.0\n")

    def _sampler(self, runner=None, **kwargs):
        return HostSampler(
            clock=lambda: self.now[0], proc_root=self.proc, sys_root=self.sys,
            runner=runner or self.runner,
            disk_usage=lambda _path: DiskUsage(1000, 600, 400),
            hostname_getter=lambda: "ammr-pi", cpu_count_getter=lambda: 4,
            page_size=4096, **kwargs)

    def _process(self, pid, name, ticks, starttime, rss=100):
        # fields below start at Linux /proc/<pid>/stat field 3.
        fields = ["S"] + ["0"] * 21
        fields[11] = str(ticks)
        fields[12] = "0"
        fields[19] = str(starttime)
        fields[21] = str(rss)
        self._write(f"proc/{pid}/stat", f"{pid} ({name}) " + " ".join(fields) + "\n")
        self._write(f"proc/{pid}/comm", name + "\n")

    def test_cpu_delta_and_reset_are_deterministic(self):
        sampler = self._sampler()
        self.assertIsNone(sampler.sample()["cpu_pct"])
        self._write("proc/stat", "cpu  130 0 60 910 0 0 0 0\ncpu0 1 0 0 9\n")
        self.now[0] += 2
        self.assertEqual(sampler.sample()["cpu_pct"], 40.0)
        self._write("proc/stat", "cpu  1 0 1 8 0 0 0 0\ncpu0 1 0 0 9\n")
        self.now[0] += 2
        self.assertIsNone(sampler.sample()["cpu_pct"])

    def test_process_cpu_is_whole_machine_normalized_and_pid_reuse_resets(self):
        self._process(42, "nav worker", 100, 500, rss=256)
        sampler = self._sampler()
        first = sampler.sample()["top_processes"][0]
        self.assertIsNone(first["cpu_pct"])
        self.assertEqual(first["name"], "nav worker")
        self.assertNotIn("cmdline", first)
        self._write("proc/stat", "cpu  150 0 50 900 0 0 0 0\ncpu0 1 0 0 9\n")
        self._process(42, "nav worker", 120, 500, rss=256)
        self.now[0] += 5
        self.assertEqual(sampler.sample()["top_processes"][0]["cpu_pct"], 20.0)
        self._write("proc/stat", "cpu  200 0 50 950 0 0 0 0\ncpu0 1 0 0 9\n")
        self._process(42, "new process", 5, 900, rss=128)
        self.now[0] += 5
        reused = sampler.sample()["top_processes"][0]
        self.assertIsNone(reused["cpu_pct"])
        self.assertEqual(reused["name"], "new process")

    def test_top_processes_are_limited_to_six(self):
        for pid in range(10, 18):
            self._process(pid, f"worker-{pid}", pid, pid)
        result = self._sampler().sample()
        self.assertEqual(len(result["top_processes"]), 6)

    def test_missing_temperature_returns_none_and_error(self):
        result = self._sampler().sample()
        self.assertIsNone(result["cpu_temp_c"])
        self.assertTrue(any(error.startswith("cpu_temp_c:")
                            for error in result["errors"]))

    def test_throttle_current_and_historical_bits_are_separate(self):
        runner = FakeRunner(throttle="throttled=0x50005\n", services=self.runner.services)
        throttle = self._sampler(runner=runner).sample()["throttle"]
        self.assertTrue(throttle["available"])
        self.assertEqual(throttle["raw"], 0x50005)
        self.assertEqual(throttle["current"], ["저전압 감지", "스로틀링"])
        self.assertEqual(throttle["historical"], ["저전압 감지", "스로틀링"])

    def test_zero_throttle_is_available_and_normal(self):
        throttle = self._sampler().sample()["throttle"]
        self.assertEqual(throttle, {"available": True, "raw": 0,
                                   "current": [], "historical": []})

    def test_disk_values_and_schema(self):
        result = self._sampler().sample()
        self.assertEqual(result["disk_pct"], 60.0)
        self.assertEqual(result["disk_free_gb"], 0.0)
        self.assertEqual(result["hostname"], "ammr-pi")
        self.assertEqual(result["cpu_count"], 4)
        self.assertEqual(result["memory_pct"], 50.0)
        self.assertEqual(result["memory_total_mb"], 1073.7)

    def test_partial_service_failure_keeps_successful_unit(self):
        def runner(args, **kwargs):
            if args[0] == "vcgencmd":
                return SimpleNamespace(returncode=0, stdout="throttled=0x0", stderr="")
            return SimpleNamespace(returncode=1, stdout=(
                "Id=jdamr-base.service\nActiveState=active\nSubState=running\n\n"),
                stderr="another unit missing")
        result = self._sampler(runner=runner).sample()
        states = {item["name"]: item["state"] for item in result["services"]}
        self.assertEqual(states["jdamr-base.service"], "active (running)")
        self.assertEqual(states["jdamr-box-rgbd.service"], "unknown")
        self.assertTrue(any(error.startswith("services:") for error in result["errors"]))
        self.assertEqual(result["swap_pct"], 25.0)

    def test_missing_commands_are_unknown_and_do_not_raise(self):
        sampler = self._sampler(runner=FakeRunner(missing=True))
        result = sampler.sample()
        self.assertEqual(result["throttle"], {
            "available": False, "raw": None, "current": [], "historical": []})
        self.assertEqual([item["name"] for item in result["services"]],
                         list(SERVICE_ALLOWLIST))
        self.assertTrue(all(item["state"] == "unknown"
                            for item in result["services"]))
        self.assertTrue(any(error.startswith("throttle:")
                            for error in result["errors"]))
        self.assertTrue(any(error.startswith("services:")
                            for error in result["errors"]))
        self.now[0] += 2
        cached = sampler.sample()
        self.assertTrue(any(error.startswith("throttle:")
                            for error in cached["errors"]))
        self.assertTrue(any(error.startswith("services:")
                            for error in cached["errors"]))

    def test_command_calls_are_allowlisted_cached_and_bounded(self):
        sampler = self._sampler()
        sampler.sample()
        self.now[0] += 9
        sampler.sample()
        self.assertEqual(len(self.runner.calls), 2)
        vcgencmd_args, vcgencmd_options = self.runner.calls[0]
        self.assertEqual(vcgencmd_args, ["vcgencmd", "get_throttled"])
        self.assertEqual(vcgencmd_options["timeout"], 0.5)
        self.assertFalse(vcgencmd_options["shell"])
        systemctl_args, options = self.runner.calls[1]
        self.assertEqual(systemctl_args[-4:], list(SERVICE_ALLOWLIST))
        self.assertEqual(options["timeout"], 1.0)
        self.assertFalse(options["shell"])
        self.assertEqual(
            [item["state"] for item in sampler.sample()["services"]],
            ["active (running)"] * 4)


if __name__ == "__main__":
    unittest.main()
