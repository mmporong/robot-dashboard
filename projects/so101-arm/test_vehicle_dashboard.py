#!/usr/bin/env python3
"""차량 대시보드의 depth-free 계약과 현황 집계를 오프라인 검증한다."""
import csv
import http.client
import json
import os
import pathlib
import queue
import re
import subprocess
import sys
import tempfile
import threading
import time
from contextlib import contextmanager
from http.server import ThreadingHTTPServer

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
MODE = os.environ.get('SO101_DASHBOARD_TEST_MODE', '').strip().lower()
if MODE not in ('unit', 'integration'):
    raise SystemExit('SO101_DASHBOARD_TEST_MODE=unit 또는 integration을 지정하세요')
if MODE == 'unit':
    from test_support.canonical_fixture import install as install_canonical_fixture
    install_canonical_fixture()
else:
    canonical_dir = os.environ.get('SO101_CANONICAL_DIR')
    if not canonical_dir or not pathlib.Path(canonical_dir).is_dir():
        raise SystemExit('integration mode에는 유효한 SO101_CANONICAL_DIR이 필요합니다')
import project as P                                           # noqa: E402
import read_runs as R                                         # noqa: E402
import panel_server as S                                      # noqa: E402
from test_support.redirect_contract import (                  # noqa: E402
    has_exact_arm_gui_prefix, has_exact_python_prefix,
    has_exact_shell_prefix, is_approved_redirect_helper,
    is_approved_offline_test, is_approved_static_tool, is_exact_blocked_shell,
)

LEGACY_REDIRECT_PY = {name: name for name in (
    'align_y.py', 'arm_lib.py', 'astra.py', 'cam_calib.py', 'cam_servo.py',
    'collect_peaks.py', 'drop_to_box.py',
    'floor_from_depth.py', 'handeye.py', 'jog_test.py', 'park.py',
    'ik_verify.py', 'pick_demo.py', 'pick_red.py', 'place_down.py', 'probe_floor.py',
    'scan_motors.py', 'servo_calib.py', 'servo_check.py', 'servo_id.py',
    'sim/mirror_daemon.py', 'sweep_x.py', 'teach_cube_offset.py', 'unfold_safe.py',
)}
LEGACY_REDIRECT_SH = {
    'run_demo.sh': 'run_demo.sh',
    'sim/restart_viewer.sh': 'sim/restart_viewer.sh',
}
LEGACY_EXCLUDED = {
    'log_state.py': 'dashboard demo의 loopback 상태 CSV 기록기',
    'sim/frame_fit.py': 'offline MuJoCo 좌표 적합 도구',
    'sim/gen_ref_poses.py': 'offline 기준 자세 생성 도구',
    'sim/sim_view.py': 'read-only MuJoCo 시각화 entrypoint',
}
LEGACY_NON_ENTRY = {
    'ds_record.py': 'dashboard dataset import support; canonical server owns runtime import',
    'sim/sim_core.py': 'MuJoCo math/render support module; no process entrypoint',
}


class FakeWorker:
    def __init__(self):
        self.cmd = queue.Queue()
        self.abort = threading.Event()
        self.terminal_listeners = []
        self.listener_add_count = 0
        self.shutdown_calls = []
        self.commands = {}
        self.submissions = []
        self.sequence = 0
        self.profile = {'state_max_age_s': 0.5}

    def submit(self, op, *args):
        self.sequence += 1
        command_id = f'base-{self.sequence}'
        self.submissions.append((op, args))
        self.commands[command_id] = {
            'id': command_id, 'op': op, 'status': 'accepted',
            'applied_action': None, 'reason': None,
        }
        return command_id

    def command_status(self, command_id):
        command = self.commands.get(command_id)
        return dict(command) if command else None

    def add_terminal_listener(self, callback):
        self.listener_add_count += 1
        self.terminal_listeners.append(callback)

        def unsubscribe():
            if callback in self.terminal_listeners:
                self.terminal_listeners.remove(callback)
        return unsubscribe

    def emit_terminal(self, status):
        for callback in list(self.terminal_listeners):
            callback(dict(status))

    def shutdown(self, reason='worker shutdown', timeout=2.0):
        self.shutdown_calls.append((reason, timeout))
        return True

    def snapshot(self):
        pos = {joint: 0.0 for joint in S.arm_lib.JOINTS}
        pos['shoulder_pan'] = -15.6
        return {'connected': True, 'calibrated': True, 'torque': True,
                'torque_state': 'on', 'recording': False,
                'pos': pos,
                'pos_at': time.monotonic(), 'range': {}, 'log': [],
                'speed_pct': 15, 'pan_lock': -15.6, 'pan_tol': 7.0,
                'safety_ready': True, 'base_interlock_active': True,
                'base_interlock_expires_at': time.monotonic() + 1.0,
                'stop_latched': False, 'actuation_epoch': 0}


class FakeKin:
    def ik_best(self, *_args, **_kwargs):
        return [0.0] * len(S.arm_lib.JOINTS)

    def fk_pos(self, _q):
        return list(S.arm_lib.PAN0)


class FakeRecorder:
    def __init__(self):
        self.started = []
        self.start_kwargs = []
        self.commands = []
        self.shutdown_called = False

    def start_episode(self, repo_id, task, fps, **channels):
        self.start_kwargs.append(dict(channels))
        validator = channels.get('validate_capability')
        capability = channels.get('capability')
        if validator is not None:
            valid, reason = validator(capability)
            if not valid:
                return {'ok': False, 'msg': reason}
        self.started.append(repo_id)
        return {'ok': True, 'repo_id': repo_id, 'fps': fps,
                'cameras': ['wrist'], 'root': '/tmp/fake'}

    def status(self):
        return {'recording': False}

    def stop_episode(self, save=True):
        return {'ok': True, 'saved': save}

    def note_command(self, result):
        if result.get('status') != 'completed' or not result.get('applied_action'):
            return False
        self.commands.append(dict(result['applied_action']))
        return True

    def shutdown(self):
        self.shutdown_called = True
        return True


class FreshCamera:
    STALE_S = 1.0

    def ensure(self):
        return None

    def snapshot_frame(self):
        return {'jpeg': b'fresh', 'sequence': 1,
                'captured_at': time.monotonic(), 'age': 0.0, 'stale': False}


class AckWorker(FakeWorker):
    def __init__(self):
        super().__init__()
        self.commands = {}
        self.sequence = 0
        self.waited = []
        self.duration_targets = []

    def submit(self, op, *args):
        self.sequence += 1
        command_id = f'fake-{self.sequence}'
        if op == 'pose':
            status = {'id': command_id, 'op': op, 'status': 'completed',
                      'applied_action': {'shoulder_lift': 5.0}, 'reason': None}
        else:
            status = {'id': command_id, 'op': op, 'status': 'rejected',
                      'applied_action': None, 'reason': 'fake rejection'}
        self.commands[command_id] = status
        self.emit_terminal(status)
        return command_id

    def command_status(self, command_id):
        return dict(self.commands[command_id])

    def wait_command(self, command_id, timeout=2.0):
        self.waited.append((command_id, timeout))
        return self.command_status(command_id)

    def estimate_motion_duration(self, target):
        self.duration_targets.append(dict(target))
        return 4.25


class InterlockWorker:
    def __init__(self):
        self.calls = []

    def update_base_evidence(self, **evidence):
        self.calls.append(evidence)
        return {'active': True}


class PendingWorker(FakeWorker):
    """stop_and_cancel 원자 계약을 HTTP 경계에서 검증하는 fake."""

    def __init__(self):
        super().__init__()
        self.commands = {}
        self.sequence = 0
        self.stop_reasons = []
        self.stop_latched = False
        self.lock = threading.Lock()
        self.race_barrier = None

    def submit(self, op, *args):
        if self.race_barrier is not None:
            self.race_barrier.wait()
        with self.lock:
            self.sequence += 1
            command_id = f'pending-{self.sequence}'
            if op == 'rearm':
                self.stop_latched = False
                status = 'completed'
            else:
                status = 'rejected' if self.stop_latched else 'accepted'
            self.commands[command_id] = {
                'id': command_id, 'op': op, 'status': status,
                'applied_action': None,
                'reason': ('stop in progress' if status == 'rejected' else None),
            }
            if status == 'accepted':
                self.cmd.put({'id': command_id, 'op': op, 'args': tuple(args)})
            elif status in ('completed', 'rejected'):
                self.emit_terminal(self.commands[command_id])
            return command_id

    def stop_and_cancel(self, reason):
        if self.race_barrier is not None:
            self.race_barrier.wait()
        with self.lock:
            self.stop_reasons.append(reason)
            self.stop_latched = True
            self.abort.set()
            while not self.cmd.empty():
                item = self.cmd.get_nowait()
                command = self.commands[item['id']]
                if command['status'] == 'accepted':
                    command['status'] = 'rejected'
                    command['reason'] = reason
                    self.emit_terminal(command)
            self.sequence += 1
            command_id = f'pending-{self.sequence}'
            terminal = {'id': command_id, 'op': 'stop', 'status': 'completed',
                        'applied_action': {}, 'reason': None}
            self.commands[command_id] = terminal
            self.emit_terminal(terminal)
            return command_id

    def wait_command(self, command_id, timeout=2.0):
        del timeout
        command = self.commands.get(command_id)
        return dict(command) if command else None

    def command_status(self, command_id):
        command = self.commands.get(command_id)
        return dict(command) if command else None


class DeferredWorker(FakeWorker):
    def __init__(self):
        super().__init__()
        self.commands = {}
        self.sequence = 0
        self.now = 0.0

    def submit(self, op, *args):
        self.sequence += 1
        command_id = f'deferred-{self.sequence}'
        self.commands[command_id] = {
            'id': command_id, 'op': op, 'args': args, 'status': 'accepted',
            'accepted_at': self.now, 'applied_action': None, 'reason': None,
        }
        return command_id

    def advance(self, seconds):
        self.now += float(seconds)

    def complete(self, command_id, applied_action=None, rejected=False):
        status = self.commands[command_id]
        status['status'] = 'rejected' if rejected else 'completed'
        status['reason'] = 'fake rejection' if rejected else None
        status['applied_action'] = None if rejected else dict(applied_action or {})
        status['updated_at'] = self.now
        self.emit_terminal(status)

    def wait_command(self, command_id, timeout=2.0):
        del timeout
        return dict(self.commands[command_id])

    def command_status(self, command_id):
        return dict(self.commands[command_id])

    def estimate_motion_duration(self, _target):
        return 1.0

    def shutdown(self, reason='worker shutdown', timeout=2.0):
        for status in self.commands.values():
            if status['status'] not in ('completed', 'rejected'):
                status['status'] = 'rejected'
                status['reason'] = reason
                self.emit_terminal(status)
        return super().shutdown(reason, timeout)


@contextmanager
def fake_panel(cam=None, worker=None, recorder=None, mirror=None):
    worker = FakeWorker() if worker is None else worker
    recorder = FakeRecorder() if recorder is None else recorder
    handler = S.make_handler(worker, FakeKin(), cam, mir=mirror, rec=recorder)
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server.server_address[1], worker, recorder
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2)
        handler.remove_terminal_listener()


def request(port, method, path, body=None, headers=None):
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=2)
    raw = body if isinstance(body, bytes) else (
        json.dumps(body).encode() if body is not None else None)
    conn.request(method, path, body=raw, headers=headers or {})
    response = conn.getresponse()
    data = response.read()
    conn.close()
    return response.status, response.getheaders(), data


def test_vehicle_channels():
    active = [k for k, v in P.CHANNELS.items() if not v.get('legacy')]
    legacy = [k for k, v in P.CHANNELS.items() if v.get('legacy')]
    assert active == ['wrist', 'sim', 'screen'], active
    assert legacy == ['rgb', 'depth'], legacy
    assert '/*__STATUS__*/' in P.DATA


def test_status_evidence():
    old_yolo, old_pick = P.YOLO_LOG, P.PICK_LOG
    try:
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            P.YOLO_LOG = root / 'yolo.jsonl'
            P.PICK_LOG = root / 'pick.csv'
            P.YOLO_LOG.write_text('\n'.join(json.dumps({'target': t}) for t in
                                             ({'confidence': .7}, None,
                                              {'confidence': .8})))
            with P.PICK_LOG.open('w', newline='') as fh:
                w = csv.DictWriter(fh, fieldnames=['ts', 'repo', 'result', 'reason'])
                w.writeheader()
                w.writerow({'ts': 't1', 'repo': 'bench',
                            'result': 'success', 'reason': ''})
                w.writerow({'ts': 't2', 'repo': 'so101_car',
                            'result': 'fail', 'reason': 'gate'})
            status = R.read_status()
    finally:
        P.YOLO_LOG, P.PICK_LOG = old_yolo, old_pick
    assert status['yolo'] == {'frames': 3, 'hits': 2, 'rate_pct': 66.7,
                              'confidence': 0.40, 'gate': 'blocked'}
    assert status['pick']['cycles'] == 1 and status['pick']['success'] == 0
    assert status['pick']['last_reason'] == 'gate'


def test_live_panel_contract():
    page = (HERE / 'panel.html').read_text()
    server = (HERE / 'panel_server.py').read_text()
    assert 'src="/depth"' not in page and 'src="/rgb"' not in page
    assert 'value="so101_car"' in page
    assert '팬 잠금 없음' in page and '상태 끊김' in page
    assert 'class Depth(' not in server
    assert "self.path == '/blob'" not in server
    assert "self.path == '/depth'" not in server
    assert "self.path == '/rgb'" not in server
    assert "depth=False, pointmap=False" in server
    assert "'mirror_piece'" in server
    assert 'new AbortController()' in page
    assert "#joints input[type=range]" in page
    assert 'performance.now()-lastStateAt>1000' in page
    assert 'X-SO101-CSRF' in page
    assert 'st.safety_ready' in page and "id=\"hCmd\"" in page
    assert 'st.base_interlock_active' in page and "id=\"hBase\"" in page
    assert 's.disabled=!motionReady' in page
    assert 'setRecorderUnavailable' in page
    assert "$('bRec').disabled=true" in page
    assert 'camera&&camera.available&&!camera.stale' in page
    assert 'st.safety_ready' in page and 'st.base_interlock_active' in page
    assert "$('bRec').disabled=recOn ? !recStatusKnown : !recordStartReady" in page
    assert '손목캠 새 프레임과 안전 상태를 확인하세요' in page
    assert "id=\"bRearm\"" in page and page.count("cmd({op:'rearm'})") == 1
    assert '&&!st.stop_latched&&!st.recording' in page
    assert "row.innerHTML" not in page
    assert "$('dsout').innerHTML" not in page
    assert 'valid_repo_id' in server
    assert 'estimate_motion_duration' in server
    assert 'worker._profile_vel' not in server
    assert 'HOME_Q' not in server
    assert 'worker.cmd.put' not in server
    if MODE == 'integration':
        assert hasattr(S.Worker, 'estimate_motion_duration')
        assert hasattr(S.Worker, 'wait_command')
        assert hasattr(S.Worker, 'stop_and_cancel')
        assert hasattr(S.Worker, 'add_terminal_listener')
        assert hasattr(S.Worker, 'shutdown')
        assert hasattr(S.Worker, '_do_rearm')
    assert 'SO101_CANONICAL_DIR' in server
    assert 'worker.cmd.get_nowait()' not in server
    assert 'record_completed_command' not in server
    assert "submit('stop')" not in server
    assert 'cancel_pending(' not in server


def test_unit_fixture_rejects_unexpected_canonical_calls():
    if MODE != 'unit':
        return
    try:
        S.arm_lib.load_kinematics()
        raise AssertionError('unit fixture가 예상 밖 canonical 호출을 허용함')
    except AssertionError as exc:
        assert 'integration mode' in str(exc)


def test_legacy_runtime_entrypoints_are_redirected_or_blocked():
    tools = HERE / 'tools'
    helper = (tools / '_canonical_redirect.py').read_text()
    assert is_approved_redirect_helper(helper)
    for relative, target in LEGACY_REDIRECT_PY.items():
        source = (tools / relative).read_text()
        assert has_exact_python_prefix(source, relative, target), relative
    for relative, target in LEGACY_REDIRECT_SH.items():
        source = (tools / relative).read_text()
        assert has_exact_shell_prefix(source, relative, target), relative
    arm_gui = tools / 'arm_gui.py'
    assert not arm_gui.is_symlink()
    assert '/home/lim/so101_tools' not in arm_gui.read_text()
    usb = (tools / 'usb_port_cycle.sh').read_text().splitlines()[:4]
    assert any('legacy 실행 차단' in line for line in usb)
    assert any('exit 2' in line for line in usb)
    assert all((tools / relative).exists() for relative in LEGACY_EXCLUDED)
    for path in tools.rglob('*'):
        if path.is_symlink():
            assert not pathlib.Path(os.readlink(path)).is_absolute(), path

    tracked = subprocess.run(
        ['git', 'ls-files', 'projects/so101-arm/tools'], cwd=HERE.parents[1],
        text=True, capture_output=True, check=True).stdout.splitlines()

    def classify(relative, source):
        name = relative.as_posix()
        if name in LEGACY_REDIRECT_PY:
            return ('redirect' if has_exact_python_prefix(
                source, name, LEGACY_REDIRECT_PY[name]) else None)
        if name in LEGACY_REDIRECT_SH:
            return ('redirect' if has_exact_shell_prefix(
                source, name, LEGACY_REDIRECT_SH[name]) else None)
        if name == '_canonical_redirect.py':
            return 'redirect-helper' if is_approved_redirect_helper(source) else None
        if name == 'arm_gui.py':
            return 'redirect' if has_exact_arm_gui_prefix(source) else None
        if name == 'usb_port_cycle.sh':
            return 'blocked' if is_exact_blocked_shell(source) else None
        if name in LEGACY_NON_ENTRY:
            return 'non-entry' if is_approved_static_tool(source, name) else None
        if name in LEGACY_EXCLUDED:
            return ('dashboard-owned-nonhardware'
                    if is_approved_static_tool(source, name) else None)
        if relative.suffix == '.py' and relative.name.startswith('test_'):
            return ('offline-test'
                    if is_approved_offline_test(source, name) else None)
        return None

    unclassified = []
    for repo_path in tracked:
        relative = pathlib.Path(repo_path).relative_to('projects/so101-arm/tools')
        if relative.suffix not in ('.py', '.sh'):
            continue
        source = (tools / relative).read_text()
        if classify(relative, source) is None:
            unclassified.append(relative.as_posix())
    assert unclassified == [], unclassified

    assert classify(
        pathlib.Path('new_silent_tool.py'),
        '#!/usr/bin/env python3\nprint("top-level side effect")\n') is None

    for name in ('log_state.py', 'ds_record.py', 'sim/test_sim_mirror.py'):
        source = (tools / name).read_text()
        hostile = 'open("/tmp/hidden-side-effect", "w").close()\n' + source
        assert classify(pathlib.Path(name), hostile) is None, name

    root = (tools / 'align_y.py').read_text()
    nested = (tools / 'sim/mirror_daemon.py').read_text()
    shell = (tools / 'run_demo.sh').read_text()
    assert not has_exact_python_prefix(
        root.replace('\n', '\nopen("/tmp/side-effect", "w").close()\n', 1),
        'align_y.py', 'align_y.py')
    assert not has_exact_python_prefix(
        root.replace('from _canonical_redirect', ' from _canonical_redirect', 1),
        'align_y.py', 'align_y.py')
    assert not has_exact_python_prefix(
        root.replace("'align_y.py'", "'./align_y.py'", 1),
        'align_y.py', 'align_y.py')
    assert not has_exact_python_prefix(
        nested.replace('.parents[1]', '.parents[0]', 1),
        'sim/mirror_daemon.py', 'sim/mirror_daemon.py')
    assert not has_exact_shell_prefix(
        shell.replace('\n', '\nprintf side-effect >/tmp/side-effect\n', 1),
        'run_demo.sh', 'run_demo.sh')
    assert not has_exact_shell_prefix(
        shell.replace(' run_demo.sh ', ' ./run_demo.sh ', 1),
        'run_demo.sh', 'run_demo.sh')
    assert not is_approved_redirect_helper(
        helper.replace('\n', '\nopen("/tmp/side-effect", "w").close()\n', 1))

    targets = set(LEGACY_REDIRECT_PY.values()) | set(LEGACY_REDIRECT_SH.values())
    targets.add('arm_gui.py')
    if MODE == 'integration':
        canonical = pathlib.Path(os.environ['SO101_CANONICAL_DIR'])
        missing = sorted(target for target in targets
                         if not (canonical / target).is_file())
        assert missing == [], missing

    probe = tools / 'unfold_safe.py'
    imported = subprocess.run(
        [sys.executable, '-c',
         ('import importlib.util,sys; '
          f'sys.path.insert(0,{str(tools)!r}); '
          f's=importlib.util.spec_from_file_location("legacy_probe",{str(probe)!r}); '
          'm=importlib.util.module_from_spec(s); s.loader.exec_module(m)')],
        text=True, capture_output=True, timeout=5)
    assert imported.returncode != 0
    assert 'legacy module import 차단' in imported.stderr


def test_legacy_redirect_executes_only_temporary_canonical_target():
    tools = HERE / 'tools'
    with tempfile.TemporaryDirectory() as tmp:
        canonical = pathlib.Path(tmp)
        targets = set(LEGACY_REDIRECT_PY.values()) | set(LEGACY_REDIRECT_SH.values())
        targets.add('arm_gui.py')
        for target in targets:
            path = canonical / target
            path.parent.mkdir(parents=True, exist_ok=True)
            if path.suffix == '.sh':
                path.write_text(f'#!/usr/bin/env bash\nprintf "TARGET:{target}\\n"\n')
            else:
                path.write_text(f'print("TARGET:{target}")\n')
        env = dict(os.environ, SO101_CANONICAL_DIR=str(canonical))
        cases = [
            ([sys.executable, str(tools / relative)], f'TARGET:{target}')
            for relative, target in LEGACY_REDIRECT_PY.items()
        ]
        cases.extend(
            (['bash', str(tools / relative)], f'TARGET:{target}')
            for relative, target in LEGACY_REDIRECT_SH.items()
        )
        cases.append(
            ([sys.executable, str(tools / 'arm_gui.py')], 'TARGET:arm_gui.py'))
        for command, expected in cases:
            result = subprocess.run(command, env=env, text=True,
                                    capture_output=True, timeout=5)
            assert result.returncode == 0, (command, result.stderr)
            assert result.stdout.strip() == expected, (command, result.stdout)

        missing = pathlib.Path(tmp) / 'missing'
        missing.mkdir()
        blocked = subprocess.run(
            [sys.executable, str(tools / 'unfold_safe.py')],
            env=dict(os.environ, SO101_CANONICAL_DIR=str(missing)),
            text=True, capture_output=True, timeout=5)
        assert blocked.returncode != 0 and 'legacy 실행 차단' in blocked.stderr

    usb = subprocess.run(['sh', str(tools / 'usb_port_cycle.sh')],
                         text=True, capture_output=True, timeout=5)
    assert usb.returncode == 2 and 'legacy 실행 차단' in usb.stderr


def test_ci_has_honest_unit_and_token_gated_integration():
    workflow = (HERE.parents[1] / '.github/workflows/so101-arm-offline.yml').read_text()
    assert 'dashboard-unit:' in workflow and 'canonical-integration:' in workflow
    assert 'SO101_DASHBOARD_TEST_MODE: unit' in workflow
    assert 'SO101_DASHBOARD_TEST_MODE: integration' in workflow
    assert 'SO101_CANONICAL_TOKEN secret 없음' in workflow
    assert 'token: ${{ secrets.SO101_CANONICAL_TOKEN }}' in workflow
    assert 'github.token' not in workflow and '||' not in workflow


def test_canonical_directory_environment_override():
    with tempfile.TemporaryDirectory() as tmp:
        canonical = pathlib.Path(tmp) / 'canonical'
        canonical.mkdir()
        (canonical / 'arm_lib.py').write_text('')
        (canonical / 'ds_record.py').write_text('')
        (canonical / 'arm_gui.py').write_text('class Worker: pass\n')
        (canonical / 'ros_base_monitor.py').write_text('class BaseMonitor: pass\n')
        env = dict(os.environ, SO101_CANONICAL_DIR=str(canonical))
        result = subprocess.run(
            [sys.executable, '-c',
             'import panel_server; print(panel_server.TOOLS)'],
            cwd=HERE, env=env, text=True, capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(canonical.resolve())


def test_http_command_boundary():
    with fake_panel() as (port, worker, recorder):
        status, _headers, page = request(port, 'GET', '/')
        assert status == 200
        match = re.search(rb'const CSRF_TOKEN="([^"]+)"', page)
        assert match, page[:200]
        token = match.group(1).decode()
        json_headers = {'Content-Type': 'application/json'}

        status, _, _ = request(port, 'POST', '/cmd', {'op': 'speed', 'pct': 12},
                               json_headers)
        assert status == 200 and worker.submissions == [('speed', (12,))]
        status, _, raw = request(port, 'POST', '/cmd',
                               {'op': 'pose', 'joints': {'shoulder_lift': 1}},
                               json_headers)
        response = json.loads(raw)
        assert status == 200 and response['status'] == 'accepted'
        assert worker.submissions[-1] == ('pose', ({'shoulder_lift': 1},))

        command_id = response['command_id']
        worker.commands[command_id].update(
            status='completed', epoch=7,
            applied_action={'shoulder_lift': 1.0})
        status, _, raw = request(port, 'GET', f'/command?id={command_id}')
        command = json.loads(raw)
        assert status == 200
        assert command == worker.commands[command_id]
        for path, expected in (
                ('/command', 400), ('/command?id=', 400),
                ('/command?id=missing', 404), ('/state?ignored=1', 400)):
            status, _, _ = request(port, 'GET', path)
            assert status == expected, (path, status)

        origin = f'http://127.0.0.1:{port}'
        browser = dict(json_headers, Origin=origin, **{'X-SO101-CSRF': token})
        status, _, _ = request(port, 'POST', '/cmd', {'op': 'speed', 'pct': 13},
                               browser)
        assert status == 200 and worker.submissions[-1] == ('speed', (13,))

        rejected = [
            ({'Content-Type': 'text/plain'}, 415),
            (dict(json_headers, Origin=f'http://localhost.evil.example:{port}'), 403),
            (dict(json_headers, Origin=origin), 403),
            (dict(json_headers, Origin=origin, **{'X-SO101-CSRF': 'wrong'}), 403),
            (dict(json_headers, Host=f'evil.example:{port}'), 403),
        ]
        for headers, expected in rejected:
            status, _, _ = request(port, 'POST', '/cmd', {'op': 'speed', 'pct': 99},
                                   headers)
            assert status == expected, (headers, status)
            assert worker.cmd.empty()

        status, _, _ = request(port, 'GET', '/state', headers={'Host': 'localhost.evil'})
        assert status == 403

        status, _, _ = request(port, 'POST', '/cmd',
                               {'op': 'rec_start', 'repo_id': '<img_onerror=x>',
                                'task': '', 'fps': 10}, browser)
        assert status == 400 and recorder.started == []
        status, _, _ = request(port, 'POST', '/cmd',
                               {'op': 'rec_replay', 'repo_id': '../escape'}, browser)
        assert status == 400

        conn = http.client.HTTPConnection('127.0.0.1', port, timeout=2)
        conn.putrequest('POST', '/cmd')
        conn.putheader('Content-Type', 'application/json')
        conn.putheader('Content-Length', str(S.MAX_COMMAND_BYTES + 1))
        conn.endheaders()
        response = conn.getresponse()
        response.read()
        assert response.status == 413
        conn.close()
        assert worker.cmd.empty()

        attacks = [
            b'{"op":"speed","pct":NaN}',
            b'{"op":"speed","pct":Infinity}',
            b'{"op":"speed","pct":-Infinity}',
            {'op': 'torque', 'on': 'false'},
            {'op': 'range', 'start': 'true'},
            {'op': 'speed', 'pct': True},
            {'op': 'pose', 'joints': {'shoulder_lift': False}},
        ]
        for attack in attacks:
            status, _, _ = request(port, 'POST', '/cmd', attack, json_headers)
            assert status == 400, attack
            assert worker.cmd.empty()


def test_deep_json_is_deterministic_400_and_server_survives():
    headers = {'Content-Type': 'application/json'}
    deeply_nested = (b'{"op":"speed","pct":' + b'[' * 1000 + b'0'
                     + b']' * 1000 + b'}')
    over_limit = (b'{"op":"speed","pct":' + b'[' * (S.MAX_JSON_DEPTH + 1)
                  + b'0' + b']' * (S.MAX_JSON_DEPTH + 1) + b'}')
    with fake_panel() as (port, worker, _):
        for body in (deeply_nested, over_limit):
            status, _, raw = request(port, 'POST', '/cmd', body, headers)
            assert status == 400, (status, raw[:200])
            assert worker.submissions == []
        status, _, raw = request(
            port, 'POST', '/cmd', {'op': 'speed', 'pct': 12}, headers)
        assert status == 200, raw
        assert worker.submissions == [('speed', (12,))]


def test_direct_rec_start_is_server_authoritative_and_fail_closed():
    payload = {'op': 'rec_start', 'repo_id': 'so101_car', 'task': 'pick',
               'fps': 10, 'wrist': True}
    headers = {'Content-Type': 'application/json'}

    worker, recorder = FakeWorker(), FakeRecorder()
    with fake_panel(cam=FreshCamera(), worker=worker, recorder=recorder) as (port, _, _):
        status, _, raw = request(port, 'POST', '/cmd', payload, headers)
    assert status == 200, raw
    assert recorder.started == ['so101_car']
    capability = recorder.start_kwargs[0]['capability']
    assert capability['actuation_epoch'] == 0
    assert capability['base_lease']['active'] is True
    assert capability['camera']['sequence'] == 1
    assert capability['pan'] == {
        'actual': -15.6, 'center': -15.6, 'tolerance': 7.0}
    assert callable(recorder.start_kwargs[0]['validate_capability'])

    class UnsafeWorker(FakeWorker):
        def __init__(self, mutation):
            super().__init__()
            self.mutation = mutation

        def snapshot(self):
            snapshot = super().snapshot()
            self.mutation(snapshot)
            return snapshot

    unsafe_mutations = [
        lambda state: state.pop('safety_ready'),
        lambda state: state.update(base_interlock_active=False),
        lambda state: state.update(
            base_interlock_expires_at=time.monotonic() - 1.0),
        lambda state: state.update(stop_latched=True),
        lambda state: state.pop('actuation_epoch'),
        lambda state: state.update(torque=False, torque_state='off'),
        lambda state: state.update(pos_at=time.monotonic() - 10.0),
        lambda state: state['pos'].pop('shoulder_lift'),
        lambda state: state.update(pan_lock=None),
        lambda state: state['pos'].update(shoulder_pan=0.0),
    ]
    for mutation in unsafe_mutations:
        worker, recorder = UnsafeWorker(mutation), FakeRecorder()
        with fake_panel(cam=FreshCamera(), worker=worker,
                        recorder=recorder) as (port, _, _):
            status, _, raw = request(port, 'POST', '/cmd', payload, headers)
        response = json.loads(raw)
        assert status == 503 and response['ok'] is False, response
        assert recorder.started == []

    class StaleCamera(FreshCamera):
        def snapshot_frame(self):
            return {'jpeg': None, 'sequence': 1,
                    'captured_at': time.monotonic() - 2.0,
                    'age': 2.0, 'stale': True}

    recorder = FakeRecorder()
    with fake_panel(cam=StaleCamera(), recorder=recorder) as (port, _, _):
        status, _, raw = request(port, 'POST', '/cmd', payload, headers)
        assert status == 503 and recorder.started == [], raw
        for op in ('rec_stop', 'rec_cancel'):
            status, _, raw = request(port, 'POST', '/cmd', {'op': op}, headers)
            assert status == 200, (op, raw)

    recorder = FakeRecorder()
    no_wrist = dict(payload, wrist=False)
    with fake_panel(cam=FreshCamera(), recorder=recorder) as (port, _, _):
        status, _, raw = request(port, 'POST', '/cmd', no_wrist, headers)
    assert status == 503 and recorder.started == [], raw

    class TransitionWorker(FakeWorker):
        def __init__(self, mutation):
            super().__init__()
            self.mutation = mutation
            self.snapshots = 0

        def snapshot(self):
            state = super().snapshot()
            self.snapshots += 1
            if self.snapshots >= 2:
                self.mutation(state)
            return state

    transition_mutations = [
        lambda state: state.update(stop_latched=True, actuation_epoch=1),
        lambda state: state.update(
            base_interlock_expires_at=time.monotonic() - 0.01),
        lambda state: state['pos'].update(shoulder_pan=0.0),
    ]
    for mutation in transition_mutations:
        worker, recorder = TransitionWorker(mutation), FakeRecorder()
        with fake_panel(cam=FreshCamera(), worker=worker,
                        recorder=recorder) as (port, _, _):
            status, _, raw = request(port, 'POST', '/cmd', payload, headers)
        response = json.loads(raw)
        assert status == 503 and response['ok'] is False, response
        assert recorder.started == [], response

    class TransitionCamera(FreshCamera):
        def __init__(self):
            self.snapshots = 0

        def snapshot_frame(self):
            self.snapshots += 1
            if self.snapshots >= 2:
                return {'jpeg': None, 'sequence': 1,
                        'captured_at': time.monotonic() - 2.0,
                        'age': 2.0, 'stale': True}
            return super().snapshot_frame()

    recorder = FakeRecorder()
    with fake_panel(cam=TransitionCamera(), recorder=recorder) as (port, _, _):
        status, _, raw = request(port, 'POST', '/cmd', payload, headers)
    response = json.loads(raw)
    assert status == 503 and response['ok'] is False, response
    assert recorder.started == []


def test_all_arm_control_ops_use_tracked_submit():
    worker = FakeWorker()
    requests = [
        ({'op': 'connect'}, ('connect', ())),
        ({'op': 'disconnect'}, ('disconnect', ())),
        ({'op': 'neutral'}, ('neutral', ())),
        ({'op': 'save_calib'}, ('save_calib', ())),
        ({'op': 'rearm'}, ('rearm', ())),
        ({'op': 'torque', 'on': True}, ('torque', (True,))),
        ({'op': 'range', 'start': True}, ('range', (True,))),
        ({'op': 'teleop_profile', 'on': True}, ('teleop_profile', (True,))),
        ({'op': 'pan_lock', 'on': True, 'tol': 7.0, 'center': -15.6},
         ('pan_lock', (True, 7.0, -15.6))),
        ({'op': 'grip_force', 'pct': 45}, ('grip_force', (45,))),
        ({'op': 'speed', 'pct': 20}, ('speed', (20,))),
    ]
    with fake_panel(worker=worker) as (port, _, _):
        for payload, expected in requests:
            status, _, raw = request(
                port, 'POST', '/cmd', payload,
                {'Content-Type': 'application/json'})
            response = json.loads(raw)
            assert status == 200, (payload, status, response)
            assert response['command_id']
            assert response['status'] == 'accepted'
            assert response['reason'] is None
            assert worker.submissions[-1] == expected
            assert worker.cmd.empty()

    worker.submit = None
    with fake_panel(worker=worker) as (port, _, _):
        status, _, raw = request(
            port, 'POST', '/cmd', {'op': 'speed', 'pct': 20},
            {'Content-Type': 'application/json'})
    response = json.loads(raw)
    assert status == 503 and response['ok'] is False
    assert '추적 인터페이스' in response['msg']


def test_record_capability_rejects_camera_rollback():
    class MutableCamera(FreshCamera):
        def __init__(self):
            self.sequence = 10
            self.captured_at = time.monotonic()

        def snapshot_frame(self):
            return {'jpeg': b'fresh', 'sequence': self.sequence,
                    'captured_at': self.captured_at,
                    'age': 0.0, 'stale': False}

    worker, camera = FakeWorker(), MutableCamera()
    ready, capability = S.record_start_readiness(worker, camera)
    assert ready, capability
    validate = S.make_record_capability_validator(worker, camera)
    assert validate(capability) == (True, None)
    camera.sequence += 2
    camera.captured_at += 0.01
    assert validate(capability) == (True, None)
    camera.sequence -= 1
    valid, reason = validate(capability)
    assert valid is False and '되돌아' in reason


def test_rec_start_camera_failure_is_http_503():
    class CameraRejectedRecorder(FakeRecorder):
        def start_episode(self, *_args, **_kwargs):
            return {'ok': False, 'msg': '손목캠 새 프레임 없음: stale'}

    recorder = CameraRejectedRecorder()
    with fake_panel(cam=FreshCamera(), recorder=recorder) as (port, _, _):
        status, _, raw = request(
            port, 'POST', '/cmd',
            {'op': 'rec_start', 'repo_id': 'so101_car', 'task': 'pick',
             'fps': 10, 'wrist': True},
            {'Content-Type': 'application/json'})
    response = json.loads(raw)
    assert status == 503 and response['ok'] is False
    assert '손목캠 새 프레임 없음' in response['msg']

    class LifecycleRejectedRecorder(FakeRecorder):
        def stop_episode(self, save=True):
            return {'ok': False, 'msg': f'{"stop" if save else "cancel"} 실패'}

    recorder = LifecycleRejectedRecorder()
    with fake_panel(recorder=recorder) as (port, _, _):
        for op in ('rec_stop', 'rec_cancel'):
            status, _, raw = request(
                port, 'POST', '/cmd', {'op': op},
                {'Content-Type': 'application/json'})
            response = json.loads(raw)
            assert status == 503 and response['ok'] is False, (op, response)


def test_camera_freshness_without_device():
    now = [10.0]
    cam = S.Camera(99, clock=lambda: now[0])
    cam._store_jpeg(b'jpeg')
    frame = cam.snapshot_frame()
    assert frame['jpeg'] == b'jpeg' and frame['sequence'] == 1
    assert frame['age'] == 0.0 and not frame['stale']
    now[0] += S.Camera.STALE_S + 0.01
    frame = cam.snapshot_frame()
    assert frame['jpeg'] is None and frame['stale']
    cam._store_jpeg(b'new')
    cam.ensure = lambda: None
    with fake_panel(cam) as (port, _worker, _recorder):
        status, headers, body = request(port, 'GET', '/frame.jpg')
        frame_headers = dict(headers)
        assert status == 200 and body == b'new'
        assert frame_headers['X-Frame-Sequence'] == '2'
        assert float(frame_headers['X-Frame-Age']) == 0.0
        assert float(frame_headers['X-Frame-Captured-At']) == now[0]
    cam._mark_failed()
    assert cam.snapshot_frame()['jpeg'] is None

    with fake_panel(cam) as (port, _worker, _recorder):
        status, _, state_raw = request(port, 'GET', '/state')
        camera = json.loads(state_raw)['vision']['camera']
        assert status == 200 and camera['stale'] is True
        # ensure()가 실제 스레드를 시작하지 않게 fake 처리한다.
        cam.ensure = lambda: None
        status, _, _ = request(port, 'GET', '/cam')
        assert status == 503


def test_mirror_discards_frame_when_status_is_stale():
    mirror = S.Mirror()
    with mirror.lock:
        mirror.jpeg = b'old-render'
        mirror.status = {'ok': False, 'msg': 'panel stale'}
    assert mirror.snapshot_jpeg() is None
    with mirror.lock:
        mirror.status = {'ok': True, 'msg': ''}
    assert mirror.snapshot_jpeg() == b'old-render'

    mirror.request = lambda *_args, **_kwargs: {
        'stale': True, 'msg': 'panel source stale'}
    try:
        mirror._read_fresh_frame()
        raise AssertionError('daemon stale 상태에서 frame bytes 경로를 허용함')
    except RuntimeError as exc:
        assert 'panel source stale' in str(exc)

    class Writer:
        def __init__(self):
            self.data = bytearray()

        def write(self, value):
            self.data.extend(value)

    class Handler:
        def __init__(self):
            self.wfile = Writer()
            self.responses = []
            self.headers = []

        def send_response(self, code):
            self.responses.append(code)

        def send_header(self, key, value):
            self.headers.append((key, value))

        def end_headers(self):
            pass

    frames = iter((b'fresh', None, b'must-not-be-read'))
    reads = []

    def next_frame():
        reads.append(True)
        return next(frames)

    handler = Handler()
    S.serve_mjpeg(handler, next_frame, fps=1_000_000, stop_on_empty=True)
    assert len(reads) == 2
    assert handler.responses == [200]
    assert b'fresh' in handler.wfile.data

    class StaleMirror:
        lock = threading.Lock()

        def __init__(self):
            self.status = {'ok': False, 'msg': 'panel source stale'}

        def ensure(self):
            pass

        def snapshot_jpeg(self):
            return None

        def request(self, _path):
            return {'ok': False, 'stale': True, 'msg': 'panel source stale'}

    stale = StaleMirror()
    with fake_panel(mirror=stale) as (port, _, _):
        status, _, body = request(port, 'GET', '/mirror')
        assert status == 200 and body == b''
        status, _, body = request(port, 'GET', '/mirror/state')
        state = json.loads(body)
        assert status == 200 and state['proxy']['ok'] is False
        assert state['daemon']['stale'] is True


def test_mirror_shutdown_reports_stuck_process_and_closes_log():
    class StuckProcess:
        def __init__(self):
            self.terminated = self.killed = False

        def poll(self):
            return None

        def terminate(self):
            self.terminated = True

        def kill(self):
            self.killed = True

        def wait(self, timeout):
            raise subprocess.TimeoutExpired('stuck-mirror', timeout)

    class Log:
        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True

    mirror = S.Mirror()
    process, log = StuckProcess(), Log()
    mirror._proc, mirror._log = process, log
    try:
        mirror.shutdown(0.001)
        raise AssertionError('생존 미러 프로세스를 정상 종료로 처리함')
    except RuntimeError as exc:
        assert '미러 데몬 종료 실패' in str(exc)
        assert '프로세스 생존' in str(exc)
    assert process.terminated and process.killed
    assert log.closed and mirror._log is None
    assert mirror._closing.is_set()

    clean = S.Mirror()
    assert clean.shutdown(0.001) is True

    class StuckThreadMirror(S.Mirror):
        def is_alive(self):
            return True

        def join(self, timeout=None):
            self.join_timeout = timeout

    stuck_thread = StuckThreadMirror()
    try:
        stuck_thread.shutdown(0.001)
        raise AssertionError('생존 미러 감시 스레드를 정상 종료로 처리함')
    except RuntimeError as exc:
        assert '감시 스레드 종료 timeout' in str(exc)
    assert stuck_thread.join_timeout is not None

    class StuckMirror:
        def shutdown(self, _timeout):
            raise RuntimeError('stuck mirror sentinel')

    try:
        S.shutdown_runtime(FakeWorker(), mirror=StuckMirror(), timeout=0.001)
        raise AssertionError('상위 shutdown이 미러 실패를 삼킴')
    except RuntimeError as exc:
        assert 'mirror: RuntimeError: stuck mirror sentinel' in str(exc)


def test_dataset_uses_completed_applied_action():
    worker, recorder = AckWorker(), FakeRecorder()
    with fake_panel(worker=worker, recorder=recorder) as (port, _, _):
        headers = {'Content-Type': 'application/json'}
        status, _, raw = request(port, 'POST', '/cmd',
                                 {'op': 'pose', 'joints': {'shoulder_lift': 99}},
                                 headers)
        response = json.loads(raw)
        assert status == 200 and response['command_id'] == 'fake-1'
        status, _, raw = request(port, 'POST', '/cmd',
                                 {'op': 'goto', 'joint': 'elbow_flex', 'value': 77},
                                 headers)
        rejected = json.loads(raw)
        assert status == 200 and rejected['ok'] is False
        assert rejected['status'] == 'rejected'
        assert rejected['reason'] == 'fake rejection'
        deadline = time.monotonic() + 1.0
        while len(recorder.commands) < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
    assert recorder.commands == [{'shoulder_lift': 5.0}]
    assert worker.listener_add_count == 1
    assert worker.terminal_listeners == []
    assert worker.waited == []


def test_stop_rejects_every_pending_command_through_atomic_api():
    worker = PendingWorker()
    first = worker.submit('pose', {'shoulder_lift': 5.0})
    second = worker.submit('goto', 'elbow_flex', 10.0)
    with fake_panel(worker=worker) as (port, _, _):
        status, _, raw = request(
            port, 'POST', '/cmd', {'op': 'stop'},
            {'Content-Type': 'application/json'})
    response = json.loads(raw)
    assert status == 200 and response['status'] == 'completed'
    assert worker.stop_reasons == ['운영자 정지']
    for command_id in (first, second):
        terminal = worker.wait_command(command_id)
        assert terminal['status'] == 'rejected'
        assert terminal['reason'] == '운영자 정지'
        assert terminal['applied_action'] is None
    stop = worker.wait_command(response['command_id'])
    assert stop['op'] == 'stop' and stop['status'] == 'completed'
    assert worker.cmd.empty()


def test_terminal_listener_records_late_completion_once():
    worker, recorder = DeferredWorker(), FakeRecorder()
    with fake_panel(worker=worker, recorder=recorder) as (port, _, _):
        status, _, raw = request(
            port, 'POST', '/cmd',
            {'op': 'pose', 'joints': {'shoulder_lift': 31.0}},
            {'Content-Type': 'application/json'})
        response = json.loads(raw)
        assert status == 200 and response['status'] == 'accepted'
        worker.advance(31.0)
        assert recorder.commands == []
        worker.complete(response['command_id'], {'shoulder_lift': 30.5})
        completed = worker.command_status(response['command_id'])
        assert completed['updated_at'] - completed['accepted_at'] == 31.0
        worker.emit_terminal(worker.commands[response['command_id']])
        rejected = worker.submit('goto', 'elbow_flex', 10.0)
        worker.complete(rejected, rejected=True)
    assert recorder.commands == [{'shoulder_lift': 30.5}]
    assert worker.listener_add_count == 1


def test_terminal_listener_tombstones_are_bounded_under_5000_completions():
    worker, recorder = FakeWorker(), FakeRecorder()
    handler = S.make_handler(worker, FakeKin(), None, rec=recorder)
    try:
        for index in range(5000):
            worker.emit_terminal({
                'id': f'stress-{index}', 'op': 'pose', 'status': 'completed',
                'applied_action': {'shoulder_lift': float(index)}, 'reason': None,
            })
        assert len(recorder.commands) == 5000
        assert handler.terminal_tombstone_count() == 256
        worker.emit_terminal({
            'id': 'stress-4999', 'op': 'pose', 'status': 'completed',
            'applied_action': {'shoulder_lift': 4999.0}, 'reason': None,
        })
        assert len(recorder.commands) == 5000
    finally:
        handler.remove_terminal_listener()


def test_stop_submit_barrier_leaves_no_pending_command():
    worker = PendingWorker()
    worker.race_barrier = threading.Barrier(2)
    responses = []
    with fake_panel(worker=worker) as (port, _, _):
        headers = {'Content-Type': 'application/json'}

        def post(body):
            responses.append(request(port, 'POST', '/cmd', body, headers))

        stop_thread = threading.Thread(target=post, args=({'op': 'stop'},))
        move_thread = threading.Thread(
            target=post,
            args=({'op': 'pose', 'joints': {'shoulder_lift': 1.0}},))
        stop_thread.start(); move_thread.start()
        stop_thread.join(2); move_thread.join(2)
    assert len(responses) == 2
    assert worker.stop_reasons == ['운영자 정지']
    assert worker.cmd.empty()
    assert all(status['status'] in ('completed', 'rejected')
               for status in worker.commands.values())


def test_torque_enable_stop_race_is_fail_closed_20x():
    headers = {'Content-Type': 'application/json'}
    for iteration in range(20):
        worker = PendingWorker()
        worker.race_barrier = threading.Barrier(2)
        responses = {}
        with fake_panel(worker=worker) as (port, _, _):
            def post(name, payload):
                status, _, raw = request(port, 'POST', '/cmd', payload, headers)
                responses[name] = (status, json.loads(raw))

            torque_thread = threading.Thread(
                target=post, args=('torque', {'op': 'torque', 'on': True}))
            stop_thread = threading.Thread(
                target=post, args=('stop', {'op': 'stop'}))
            torque_thread.start()
            stop_thread.start()
            torque_thread.join(2)
            stop_thread.join(2)
            assert not torque_thread.is_alive() and not stop_thread.is_alive(), iteration
            worker.race_barrier = None

            torque_response = responses['torque'][1]
            stop_response = responses['stop'][1]
            torque = worker.wait_command(torque_response['command_id'])
            stop = worker.wait_command(stop_response['command_id'])
            assert torque['op'] == 'torque' and torque['status'] == 'rejected'
            assert torque['applied_action'] is None
            assert stop['op'] == 'stop' and stop['status'] == 'completed'
            assert worker.cmd.empty()

            status, _, raw = request(
                port, 'POST', '/cmd', {'op': 'torque', 'on': True}, headers)
            post_stop = json.loads(raw)
            assert status == 200 and post_stop['status'] == 'rejected'
            assert worker.wait_command(post_stop['command_id'])['status'] == 'rejected'
            assert worker.cmd.empty()

            status, _, raw = request(
                port, 'POST', '/cmd', {'op': 'rearm'}, headers)
            rearm = json.loads(raw)
            assert status == 200 and rearm['status'] == 'completed'
            status, _, raw = request(
                port, 'POST', '/cmd', {'op': 'torque', 'on': True}, headers)
            after_rearm = json.loads(raw)
            assert status == 200 and after_rearm['status'] == 'accepted'
            queued = worker.cmd.get_nowait()
            assert queued['id'] == after_rearm['command_id'] and queued['op'] == 'torque'


def test_shutdown_runtime_terminals_commands_and_joins_components():
    worker, recorder = DeferredWorker(), FakeRecorder()
    accepted = worker.submit('pose', {'shoulder_lift': 1.0})
    executing = worker.submit('goto', 'elbow_flex', 2.0)
    worker.commands[executing]['status'] = 'executing'

    class Component:
        def __init__(self):
            self.stopped = self.joined = self.shutdown_called = False

        def stop(self):
            self.stopped = True

        def join(self, _timeout):
            self.joined = True

        def shutdown(self, _timeout=None):
            self.shutdown_called = True

    monitor, camera, mirror = Component(), Component(), Component()
    assert S.shutdown_runtime(worker, recorder, monitor, camera, mirror,
                              reason='panel shutdown', timeout=1.5)
    assert worker.wait_command(accepted)['status'] == 'rejected'
    assert worker.wait_command(executing)['status'] == 'rejected'
    assert worker.shutdown_calls == [('panel shutdown', 1.5)]
    assert monitor.stopped and monitor.joined
    assert camera.shutdown_called and mirror.shutdown_called

    class FailedRecorder(FakeRecorder):
        def shutdown(self):
            return False

    try:
        S.shutdown_runtime(DeferredWorker(), recorder=FailedRecorder())
        raise AssertionError('Recorder.shutdown False를 정상 종료로 처리함')
    except RuntimeError as exc:
        assert 'recorder: shutdown 실패' in str(exc)


def test_home_requires_configured_pose():
    worker = AckWorker()
    original = S.arm_lib.load_mapping
    S.arm_lib.load_mapping = lambda: {}
    try:
        with fake_panel(worker=worker) as (port, _, _):
            status, _, raw = request(
                port, 'POST', '/cmd', {'op': 'home'},
                {'Content-Type': 'application/json'})
    finally:
        S.arm_lib.load_mapping = original
    response = json.loads(raw)
    assert status == 503 and response['ok'] is False
    assert not worker.commands


def test_ik_requires_public_duration_estimate():
    worker = AckWorker()
    with fake_panel(worker=worker) as (port, _, _):
        status, _, raw = request(
            port, 'POST', '/cmd',
            {'op': 'ik', 'x': 0.2, 'y': 0.0, 'z': -0.05, 'pitch': -90},
            {'Content-Type': 'application/json'})
    response = json.loads(raw)
    assert status == 200 and response['command_id'] == 'fake-1'
    assert len(worker.duration_targets) == 1
    assert set(worker.duration_targets[0]) == set(S.arm_lib.JOINTS)

    worker = AckWorker()

    def stale_duration(_target):
        raise RuntimeError('state stale')
    worker.estimate_motion_duration = stale_duration
    with fake_panel(worker=worker) as (port, _, _):
        status, _, raw = request(
            port, 'POST', '/cmd',
            {'op': 'ik', 'x': 0.2, 'y': 0.0, 'z': -0.05, 'pitch': -90},
            {'Content-Type': 'application/json'})
    response = json.loads(raw)
    assert status == 503 and 'state stale' in response['msg']
    assert not worker.commands


def test_base_monitor_combines_cmd_and_odom_without_publisher():
    worker = InterlockWorker()
    monitor = S.BaseMonitor(worker)
    before = time.monotonic()
    result = monitor.submit_evidence(
        cmd=(0.0, 0.02, 10.0), odom=(0.005, 0.0, 10.1),
        publishers=[('collision_monitor', '/')],
        subscribers=[('jdamr_base_driver', '/')])
    after = time.monotonic()
    assert result == {'active': True}
    assert worker.calls == [{
        'odom_linear_mps': 0.005, 'odom_angular_rps': 0.0,
        'cmd_vel_linear_mps': 0.0, 'cmd_vel_angular_rps': 0.02,
        'cmd_vel_publishers': [('collision_monitor', '/')],
        'cmd_vel_subscribers': [('jdamr_base_driver', '/')],
        'odom_observed_at': 10.1, 'cmd_vel_observed_at': 10.0,
        'graph_observed_at': worker.calls[0]['graph_observed_at']}]
    assert before <= worker.calls[0]['graph_observed_at'] <= after
    source = (pathlib.Path(S.TOOLS) / 'ros_base_monitor.py').read_text()
    assert 'create_publisher' not in source


def test_record_dashboard_contract():
    page = (HERE / 'dashboard.tpl.html').read_text()
    assert 'const STATUS = /*__STATUS__*/{};' in page
    assert 'SO-101 Mobile Manipulation' in page
    assert "filter(j=>j!=='shoulder_pan'" in page
    assert "const j0 =" not in page


def main():
    print(f'MODE — {MODE}')
    if MODE == 'unit':
        print('INTEGRATION — SKIP (SO101_CANONICAL_TOKEN/canonical checkout 없음)')
    else:
        print(f'INTEGRATION — canonical={pathlib.Path(os.environ["SO101_CANONICAL_DIR"]).resolve()}')
    tests = [test_vehicle_channels, test_status_evidence,
             test_live_panel_contract, test_unit_fixture_rejects_unexpected_canonical_calls,
             test_legacy_runtime_entrypoints_are_redirected_or_blocked,
             test_legacy_redirect_executes_only_temporary_canonical_target,
             test_ci_has_honest_unit_and_token_gated_integration,
             test_canonical_directory_environment_override,
             test_http_command_boundary,
             test_deep_json_is_deterministic_400_and_server_survives,
             test_direct_rec_start_is_server_authoritative_and_fail_closed,
             test_record_capability_rejects_camera_rollback,
             test_all_arm_control_ops_use_tracked_submit,
             test_rec_start_camera_failure_is_http_503,
             test_camera_freshness_without_device,
             test_mirror_discards_frame_when_status_is_stale,
             test_mirror_shutdown_reports_stuck_process_and_closes_log,
             test_dataset_uses_completed_applied_action,
             test_stop_rejects_every_pending_command_through_atomic_api,
             test_terminal_listener_records_late_completion_once,
             test_terminal_listener_tombstones_are_bounded_under_5000_completions,
             test_stop_submit_barrier_leaves_no_pending_command,
             test_torque_enable_stop_race_is_fail_closed_20x,
             test_shutdown_runtime_terminals_commands_and_joins_components,
             test_home_requires_configured_pose,
             test_ik_requires_public_duration_estimate,
             test_record_dashboard_contract]
    if MODE == 'integration':
        tests.insert(-1, test_base_monitor_combines_cmd_and_odom_without_publisher)
    for test in tests:
        test()
        print(f'PASS — {test.__name__}')
    print(f'통과 — 차량 대시보드 {len(tests)}항목')


if __name__ == '__main__':
    main()
