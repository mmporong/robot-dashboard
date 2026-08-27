#!/usr/bin/env python3
"""SO-101 실기 제어 패널 서버 — 대시보드 디자인 언어를 쓰는 라이브 패널.

다른 프로젝트(capstone-pick·slam)의 대시보드는 **기록을 보는** 정적 페이지지만,
이 패널은 **실물 팔을 움직이는** 라이브 페이지라 뒤에 서버가 필요하다. 시리얼
통신은 `~/so101-mobile-manipulation/arm_gui.py`의 `Worker`(전담 스레드)를 쓰고, 이
서버는 그 앞에 HTTP 만 얹는다.

    GET  /        → panel.html
    GET  /state   → Worker 상태 JSON (연결·캘리브·토크·관절각·범위·로그)
    GET  /command?id=... → 특정 Worker 명령 상태·applied_action 원증거
    GET  /cam     → 손목캠 MJPEG
    GET  /mirror  → MuJoCo 미러 MJPEG
    POST /cmd     → {"op": "connect" | "disconnect" | "torque" | "neutral"
                     | "range" | "save_calib" | "jog" | "ik" | "home"
                     | "mirror_piece"} + 인자

IK 는 서버에서 푼다 — 캡스톤 `kinematics.ik_best` 로 관절각을 만들어 Worker 에
넘긴다. 좌표는 pan 축 기준(x=전방·y=좌·z=상, 원점=베이스 서보 회전 중심).

사용:
    conda activate lerobot
    python3 panel_server.py            # http://127.0.0.1:8765
    python3 panel_server.py --port-serial /dev/ttyACM1 --http 8766
"""
import argparse
from collections import OrderedDict
import ipaddress
import json
import math
import pathlib
import secrets
import subprocess
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

HERE = pathlib.Path(__file__).parent
TOOLS = pathlib.Path(os.environ.get(
    'SO101_CANONICAL_DIR', '~/so101-mobile-manipulation')).expanduser().resolve()
sys.path.insert(0, str(TOOLS))

import arm_lib                                    # noqa: E402
import ds_record                                  # noqa: E402  (LeRobot 표준 기록)
from arm_gui import Worker                        # noqa: E402  (시리얼 워커 재사용)
from ros_base_monitor import BaseMonitor          # noqa: E402  (읽기 전용 ROS 감시)

MAX_COMMAND_BYTES = 64 * 1024
MAX_JSON_DEPTH = 64
REPO_ID_CHARS = frozenset(
    'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-')
BOOL_FIELDS = frozenset(('on', 'start', 'wrist'))


def valid_repo_id(value):
    """로컬 디렉터리 한 칸으로만 쓸 수 있는 데이터셋 ID인지 검사한다."""
    return (isinstance(value, str) and 1 <= len(value) <= 64
            and not value.startswith(('.', '-'))
            and all(ch in REPO_ID_CHARS for ch in value))


def validate_json_values(value, key=None):
    """명령 payload를 반복 순회해 깊이·비유한 수·bool 혼동을 거부한다."""
    pending = [(value, key, 0)]
    while pending:
        current, current_key, depth = pending.pop()
        if depth > MAX_JSON_DEPTH:
            raise ValueError(f'JSON 중첩은 {MAX_JSON_DEPTH}단계 이하여야 합니다')
        if current_key in BOOL_FIELDS and type(current) is not bool:
            raise ValueError(f'{current_key}는 JSON boolean이어야 합니다')
        if type(current) is bool:
            if current_key not in BOOL_FIELDS:
                raise ValueError(
                    f'{current_key or "값"}에는 boolean을 사용할 수 없습니다')
            continue
        if isinstance(current, (int, float)):
            if not math.isfinite(current):
                raise ValueError(f'{current_key or "값"}은 유한한 수여야 합니다')
            continue
        if isinstance(current, dict):
            pending.extend((child, child_key, depth + 1)
                           for child_key, child in current.items())
        elif isinstance(current, list):
            pending.extend((child, None, depth + 1) for child in current)


def reject_json_constant(value):
    raise ValueError(f'비표준 숫자 {value}')


def record_start_readiness(worker, cam, *, clock=time.monotonic):
    """차량 기록 시작에 필요한 감사 가능한 capability를 발급한다."""
    ready, evidence = _record_capability_state(worker, cam, clock=clock,
                                               ensure_camera=True)
    if not ready:
        return False, evidence
    snapshot, frame, now = evidence
    pos = snapshot['pos']
    return True, {
        'issued_at': now,
        'actuation_epoch': snapshot['actuation_epoch'],
        'base_lease': {
            'active': True,
            'expires_at': float(snapshot['base_interlock_expires_at']),
        },
        'camera': {
            'sequence': int(frame['sequence']),
            'captured_at': float(frame['captured_at']),
        },
        'pan': {
            'actual': float(pos['shoulder_pan']),
            'center': float(snapshot['pan_lock']),
            'tolerance': float(snapshot['pan_tol']),
        },
        'arm': {
            'connected': True,
            'calibrated': True,
            'safety_ready': True,
            'torque_state': 'on',
            'pos_at': float(snapshot['pos_at']),
        },
    }


def _record_capability_state(worker, cam, *, clock, ensure_camera=False):
    """현재 Worker·camera 상태를 capability 검증용으로 읽는다."""
    try:
        snapshot = worker.snapshot()
    except Exception as e:
        return False, f'팔 상태 확인 실패: {type(e).__name__}'
    if not isinstance(snapshot, dict):
        return False, '팔 상태 스냅샷이 유효하지 않습니다'
    required = (
        ('connected', '팔 연결 필요'),
        ('calibrated', '캘리브레이션 필요'),
        ('safety_ready', '보호 레지스터 검증 필요'),
        ('base_interlock_active', '베이스 인터록 준비 필요'),
    )
    for field, reason in required:
        if snapshot.get(field) is not True:
            return False, reason
    if snapshot.get('stop_latched') is not False:
        return False, '정지 latch 해제가 필요합니다'
    epoch = snapshot.get('actuation_epoch')
    if type(epoch) is not int or epoch < 0:
        return False, '동작 epoch 근거가 없습니다'
    if not (snapshot.get('torque') is True
            and snapshot.get('torque_state') == 'on'):
        return False, '토크 ON read-back이 필요합니다'
    pos = snapshot.get('pos')
    if (not isinstance(pos, dict)
            or any(joint not in pos for joint in arm_lib.JOINTS)):
        return False, '현재 관절 자세가 완전하지 않습니다'
    try:
        if any(type(pos[joint]) not in (int, float)
               or not math.isfinite(pos[joint]) for joint in arm_lib.JOINTS):
            return False, '현재 관절 자세가 유효하지 않습니다'
        observed_at = float(snapshot.get('pos_at'))
        base_expires_at = float(snapshot.get('base_interlock_expires_at'))
        max_age = float((getattr(worker, 'profile', None) or {})['state_max_age_s'])
        now = clock()
        age = now - observed_at
    except (KeyError, TypeError, ValueError, OverflowError):
        return False, '관절 자세 freshness 근거가 없습니다'
    if not math.isfinite(age) or not math.isfinite(max_age) \
            or max_age <= 0 or age < 0 or age > max_age:
        return False, '현재 관절 자세가 오래되었습니다'
    if not math.isfinite(base_expires_at) or base_expires_at <= now:
        return False, '베이스 인터록 lease가 만료되었습니다'
    try:
        pan_center = float(snapshot.get('pan_lock'))
        pan_tol = float(snapshot.get('pan_tol'))
        pan_actual = float(pos['shoulder_pan'])
    except (KeyError, TypeError, ValueError, OverflowError):
        return False, '차량 팬 잠금 근거가 없습니다'
    if (not math.isfinite(pan_center) or not math.isfinite(pan_tol)
            or pan_tol <= 0 or not math.isfinite(pan_actual)):
        return False, '차량 팬 잠금 근거가 유효하지 않습니다'
    if abs(pan_actual - pan_center) > pan_tol:
        return False, '실제 shoulder_pan이 차량 팬 허용 범위를 벗어났습니다'
    if cam is None:
        return False, '손목캠이 필요합니다'
    try:
        if ensure_camera:
            cam.ensure()
        frame = cam.snapshot_frame()
    except Exception as e:
        return False, f'손목캠 상태 확인 실패: {type(e).__name__}'
    if not isinstance(frame, dict):
        return False, '손목캠 상태가 유효하지 않습니다'
    try:
        sequence = int(frame.get('sequence'))
        captured_at = float(frame.get('captured_at'))
        frame_age = float(frame.get('age'))
        stale_limit = float(getattr(cam, 'STALE_S'))
    except (TypeError, ValueError, OverflowError):
        return False, '손목캠 fresh-frame 근거가 없습니다'
    if (frame.get('stale') is not False or sequence <= 0
            or not math.isfinite(captured_at) or not math.isfinite(frame_age)
            or not math.isfinite(stale_limit) or stale_limit <= 0
            or frame_age < 0 or frame_age > stale_limit
            or not isinstance(frame.get('jpeg'), bytes) or not frame['jpeg']):
        return False, '손목캠 새 프레임이 필요합니다'
    return True, (snapshot, frame, now)


def make_record_capability_validator(worker, cam, *, clock=time.monotonic):
    """발급 capability의 epoch·lease·팬·카메라 high-water를 재검증한다."""
    high_water = {'sequence': None, 'captured_at': None}
    high_water_lock = threading.Lock()

    def validate(capability):
        if not isinstance(capability, dict):
            return False, '기록 capability가 유효하지 않습니다'
        ready, evidence = _record_capability_state(worker, cam, clock=clock)
        if not ready:
            return False, evidence
        snapshot, frame, _now = evidence
        try:
            epoch = capability['actuation_epoch']
            lease_expires = float(capability['base_lease']['expires_at'])
            camera_sequence = int(capability['camera']['sequence'])
            camera_captured_at = float(capability['camera']['captured_at'])
            pan_center = float(capability['pan']['center'])
            pan_tol = float(capability['pan']['tolerance'])
            current_lease = float(snapshot['base_interlock_expires_at'])
            current_sequence = int(frame['sequence'])
            current_captured_at = float(frame['captured_at'])
        except (KeyError, TypeError, ValueError, OverflowError):
            return False, '기록 capability 근거가 불완전합니다'
        if snapshot['actuation_epoch'] != epoch:
            return False, 'STOP 이후 동작 epoch가 변경되었습니다'
        if current_lease < lease_expires:
            return False, '베이스 인터록 lease가 교체되거나 단축되었습니다'
        if (float(snapshot['pan_lock']) != pan_center
                or float(snapshot['pan_tol']) != pan_tol):
            return False, '차량 팬 잠금 설정이 변경되었습니다'
        if (current_sequence < camera_sequence
                or current_captured_at < camera_captured_at):
            return False, '손목캠 프레임이 capability보다 이전으로 되돌아갔습니다'
        with high_water_lock:
            previous_sequence = high_water['sequence']
            previous_captured_at = high_water['captured_at']
            if (previous_sequence is not None
                    and (current_sequence < previous_sequence
                         or current_captured_at < previous_captured_at)):
                return False, '손목캠 프레임 순서가 되돌아갔습니다'
            high_water['sequence'] = current_sequence
            high_water['captured_at'] = current_captured_at
        return True, None

    return validate


def _loopback(value):
    try:
        return ipaddress.ip_address(value).is_loopback
    except ValueError:
        return value == 'localhost'


def _host_port(value, default_port):
    """Host 헤더를 URL 파서로 엄격히 분리한다. 사용자정보·경로는 허용하지 않는다."""
    if not value or any(ch in value for ch in '/?#@'):
        return None
    try:
        parsed = urlsplit('//' + value)
        host = parsed.hostname
        port = parsed.port if parsed.port is not None else default_port
    except ValueError:
        return None
    if not host or not _loopback(host):
        return None
    return host.lower(), port


def _same_origin(value, server_port):
    try:
        parsed = urlsplit(value)
        port = parsed.port if parsed.port is not None else 80
    except ValueError:
        return False
    return (parsed.scheme == 'http' and parsed.username is None
            and parsed.password is None and not parsed.path
            and not parsed.query and not parsed.fragment
            and parsed.hostname is not None and _loopback(parsed.hostname)
            and port == server_port)


class Camera(threading.Thread):
    """UVC 카메라를 10fps로 읽어 최신 JPEG 한 장만 유지한다.

    클라이언트 수와 무관하게 캡처는 한 스레드가 하고, /cam 스트림들은 이 버퍼를
    나눠 읽는다. 첫 요청이 올 때까지 카메라를 열지 않는다(게으른 시작) — 패널만
    띄우고 캠을 안 볼 때 USB 대역·CPU를 안 쓰기 위해서다.
    """

    STALE_S = 1.0

    def __init__(self, index, clock=time.monotonic):
        super().__init__(daemon=True)
        self.index = index
        self.lock = threading.Lock()
        self.jpeg = None
        self.sequence = 0
        self.captured_at = None
        self._clock = clock
        self.started = False
        self._start_lock = threading.Lock()   # 핸들러가 병렬이라 check-then-act 보호
        self._closing = threading.Event()

    def ensure(self):
        with self._start_lock:
            if not self.started:
                self.started = True
                self.start()

    @staticmethod
    def find_index(prefer=None):
        """쓸 수 있는 UVC 노드를 고른다 — /dev/video 번호는 재연결마다 밀린다.

        내장 웹캠(ASUS FHD/IR)을 피하고 외장 UVC를 우선한다. 같은 장치가 노드를
        둘씩 잡는데(캡처+메타데이터) 실제로 열리는 것은 보통 앞 번호다.
        """
        import glob, pathlib as _pl
        cands = []
        for dev in sorted(glob.glob('/dev/video*'),
                          key=lambda s: int(s.rsplit('video', 1)[1])):
            n = _pl.Path(f'/sys/class/video4linux/{_pl.Path(dev).name}/name')
            name = n.read_text().strip() if n.exists() else ''
            idx = int(dev.rsplit('video', 1)[1])
            builtin = ('ASUS' in name or 'IR camera' in name)
            cands.append((builtin, idx, name))
        ext = [c for c in cands if not c[0]]
        # ★ 외장 UVC 가 없으면 카메라를 끈다(None). 내장 웹캠으로 조용히
        # 대체하면 패널에 노트북 화면이 떠서 손목캠으로 오인한다
        # (2026-08-20 USB 허브 이설 후 실측 — 손목캠 미검출 시 video0 을 잡았다).
        if not ext:
            return None
        if prefer is not None and any(c[1] == prefer for c in ext):
            return prefer
        return ext[0][1]

    def snapshot_jpeg(self):
        return self.snapshot_frame()['jpeg']

    def snapshot_frame(self):
        with self.lock:
            captured_at = self.captured_at
            age = (None if captured_at is None
                   else max(0.0, self._clock() - captured_at))
            stale = self.jpeg is None or age is None or age > self.STALE_S
            return {'jpeg': None if stale else self.jpeg,
                    'sequence': self.sequence,
                    'captured_at': captured_at,
                    'age': age, 'stale': stale}

    def _store_jpeg(self, jpeg):
        with self.lock:
            self.jpeg = jpeg
            self.sequence += 1
            self.captured_at = self._clock()

    def _mark_failed(self):
        with self.lock:
            self.jpeg = None
            self.captured_at = None

    def shutdown(self, timeout=2.0):
        """캡처 루프를 멈추고 카메라 핸들 해제를 bounded wait한다."""
        self._closing.set()
        if self.is_alive():
            self.join(max(0.0, float(timeout)))
        return not self.is_alive()

    def _open(self, cv2):
        cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)   # 백엔드 명시 — GStreamer 로
        if not cap.isOpened():                    # 열리면 set() 이 조용히 무시된다
            alt = self.find_index()               # 번호가 밀렸으면 다시 찾는다
            if alt is None:
                return None                       # 외장 캠이 사라짐 — 내장으로 안 간다
            if alt != self.index:
                self.index = alt
                cap = cv2.VideoCapture(alt, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        # 차량 손목캠의 검증 해상도와 맞춘다. YOLO 실측도 352×288 기준이라 패널과
        # 관찰기가 같은 픽셀 좌표를 쓰며, 불필요한 USB·인코딩 부하도 피한다.
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 352)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 288)
        fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        fmt = ''.join(chr((fcc >> 8 * k) & 0xFF) for k in range(4))
        # set() 은 실패해도 조용하다 — 협상 결과를 반드시 읽어서 남긴다
        print(f'손목캠 협상: {fmt} '
              f'{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
              f'{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} '
              f'@ {cap.get(cv2.CAP_PROP_FPS):.0f}fps', flush=True)
        return cap

    def run(self):
        import cv2
        cap = self._open(cv2)
        if cap is None:
            return
        try:
            fails = 0
            while not self._closing.is_set():
                ok, frame = cap.read()
                if ok:
                    fails = 0
                    ok2, buf = cv2.imencode('.jpg', frame,
                                            [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ok2:
                        self._store_jpeg(buf.tobytes())
                    else:
                        self._mark_failed()
                else:
                    # 케이블이 빠지면 read 가 영원히 False 다 — 방치하면 /cam 이 마지막
                    # JPEG 로 굳는다(깊이 쪽에서 없앤 바로 그 증상). 닫고 다시 연다.
                    fails += 1
                    self._mark_failed()
                    if fails >= 50:
                        cap.release()
                        if self._closing.wait(2.0):
                            return
                        cap = self._open(cv2)
                        if cap is None:
                            return
                        fails = 0
                self._closing.wait(0.1)          # 10fps — 패널 확인용이라 충분
        finally:
            cap.release()


class Mirror(threading.Thread):
    """MuJoCo 미러 데몬(~/so101-mobile-manipulation/sim/mirror_daemon.py)을 감독·중계한다.

    별도 프로세스인 이유는 깊이 데몬과 다르다 — 장치 교착이 아니라 **환경**이다.
    mujoco 는 rlwalk 환경에만 설치돼 있고 이 서버는 lerobot 환경에서 돈다.
    두 환경을 한 프로세스에 합칠 수 없으니 HTTP 로 잇는다.

    미러는 읽기 전용 표시 계층이다 — 데몬은 /state 를 읽어 그릴 뿐 팔에 명령을
    내리지 않는다. 그래서 데몬이 죽어도 팔은 아무 영향을 받지 않는다.
    """

    STALE_S = 20.0
    SIM_DIR = TOOLS / 'sim'

    def __init__(self, port=8768, piece='cube'):
        super().__init__(daemon=True)
        self.port = port
        self.piece = piece
        self.lock = threading.Lock()
        self.jpeg = None
        self.status = {'ok': False, 'msg': '시작 전'}
        self.started = False
        self._start_lock = threading.Lock()
        self._proc_lock = threading.Lock()
        self._stop_guard = threading.Lock()
        self._closing = threading.Event()
        self._proc = None
        self._log = None
        self._restarts = 0

    @staticmethod
    def python_bin():
        """mujoco 가 있는 인터프리터 — 없으면 None(미러 비활성)."""
        p = pathlib.Path.home() / 'miniforge3/envs/rlwalk/bin/python'
        return str(p) if p.exists() else None

    def snapshot_jpeg(self):
        with self.lock:
            return self.jpeg if self.status.get('ok') else None

    def ensure(self):
        with self._start_lock:
            if not self.started:
                self.started = True
                self.start()

    def shutdown(self, timeout=6.0):
        timeout = max(0.0, float(timeout))
        deadline = time.monotonic() + timeout
        self._closing.set()
        errors = []
        try:
            self._stop_proc(grace=max(0.0, deadline - time.monotonic()))
        except RuntimeError as e:
            errors.append(str(e))
        if self.is_alive():
            self.join(max(0.0, deadline - time.monotonic()))
        if self.is_alive():
            errors.append('미러 감시 스레드 종료 timeout')
        with self._proc_lock:
            process_alive = self._proc is not None and self._proc.poll() is None
        if process_alive:
            errors.append('미러 데몬 프로세스가 종료되지 않았습니다')
        if self._log is not None:
            try:
                self._log.close()
            except Exception as e:
                errors.append(f'미러 로그 닫기 실패: {type(e).__name__}: {e}')
            finally:
                self._log = None
        if errors:
            raise RuntimeError(' | '.join(errors))
        return True

    def request(self, path, body=None, timeout=4.0):
        """데몬으로 중계 (POST). 시점 변경·프리뷰·재생에 쓴다."""
        import urllib.error
        import urllib.request
        url = f'http://127.0.0.1:{self.port}{path}'
        try:
            if body is None:
                with urllib.request.urlopen(url, timeout=timeout) as r:
                    return json.loads(r.read())
            req = urllib.request.Request(
                url, method='POST', data=json.dumps(body).encode(),
                headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            try:
                return json.loads(e.read())
            except Exception:
                return {'ok': False, 'msg': f'미러 데몬 HTTP {e.code}'}
        except Exception as e:
            return {'ok': False, 'msg': f'미러 데몬 통신 실패: {type(e).__name__}'}

    def _read_fresh_frame(self):
        """데몬이 공개한 freshness 상태를 확인한 뒤에만 JPEG를 받는다."""
        daemon = self.request('/state', timeout=3.0)
        if daemon.get('stale') is not False:
            raise RuntimeError(daemon.get('msg') or '미러 데몬 프레임 stale')
        import urllib.request
        url = f'http://127.0.0.1:{self.port}/frame.jpg'
        with urllib.request.urlopen(url, timeout=3.0) as response:
            jpeg = response.read()
        if not jpeg:
            raise RuntimeError('미러 데몬 JPEG 없음')
        return jpeg

    def _spawn(self):
        if self._closing.is_set():
            return
        py = self.python_bin()
        if py is None:
            with self.lock:
                self.status = {'ok': False,
                               'msg': 'mujoco 환경(rlwalk) 없음 — 미러 비활성'}
            return
        # 포트 선점 확인 — 살아 있는 데몬이 있으면 그걸 쓴다(깊이와 같은 규약)
        h = self.request('/health', timeout=0.5)
        if h.get('beat_age') is not None:
            if h['beat_age'] <= self.STALE_S:
                return
            subprocess.run(['fuser', '-k', '-TERM', f'{self.port}/tcp'],
                           capture_output=True)
            if self._closing.wait(1.5):
                return
        with self._proc_lock:
            if self._closing.is_set():
                return
            if self._log is None:
                self._log = open(HERE / 'mirror_daemon.log', 'ab', buffering=0)
            self._proc = subprocess.Popen(
                [py, '-u', str(self.SIM_DIR / 'mirror_daemon.py'),
                 '--http', str(self.port), '--piece', self.piece],
                cwd=str(self.SIM_DIR), stdout=self._log,
                stderr=subprocess.STDOUT)

    def _stop_proc(self, grace=6.0):
        grace = max(0.0, float(grace))
        if not self._stop_guard.acquire(timeout=grace):
            raise RuntimeError('미러 데몬 종료 실패 — 다른 종료 작업 timeout')
        try:
            with self._proc_lock:
                p = self._proc
            if p is None:
                return True
            if p.poll() is not None:
                with self._proc_lock:
                    if self._proc is p:
                        self._proc = None
                return True
            errors = []
            try:
                p.terminate()
            except Exception as e:
                errors.append(f'terminate 실패: {type(e).__name__}: {e}')
            try:
                p.wait(grace)
            except Exception as e:
                errors.append(f'terminate wait 실패: {type(e).__name__}: {e}')
                try:
                    p.kill()
                except Exception as kill_error:
                    errors.append(f'kill 실패: {type(kill_error).__name__}: {kill_error}')
                try:
                    p.wait(min(3.0, grace))
                except Exception as wait_error:
                    errors.append(f'kill wait 실패: {type(wait_error).__name__}: {wait_error}')
            if p.poll() is None:
                errors.append('프로세스 생존')
            else:
                with self._proc_lock:
                    if self._proc is p:
                        self._proc = None
            if errors and p.poll() is None:
                raise RuntimeError('미러 데몬 종료 실패 — ' + ' | '.join(errors))
            return True
        finally:
            self._stop_guard.release()

    def run(self):
        self._spawn()
        last_ok = time.monotonic()
        while not self._closing.wait(0.1):
            try:
                j = self._read_fresh_frame()
                with self.lock:
                    self.jpeg = j
                    self.status = {'ok': True, 'msg': ''}
                last_ok = time.monotonic()
                self._restarts = 0
            except Exception as e:
                with self.lock:
                    self.jpeg = None
                    self.status = {'ok': False,
                                   'msg': f'미러 프레임 stale ({type(e).__name__})'}
                if time.monotonic() - last_ok > 15.0:
                    with self.lock:
                        self.status = {'ok': False,
                                       'msg': f'미러 데몬 무응답 — 재시작 ({type(e).__name__})'}
                    try:
                        self._stop_proc()
                    except RuntimeError as stop_error:
                        with self.lock:
                            self.status = {'ok': False, 'msg': str(stop_error)}
                        self._closing.set()
                        break
                    self._restarts += 1
                    if self._closing.wait(
                            min(3.0 * (2 ** min(self._restarts - 1, 3)), 30.0)):
                        break
                    self._spawn()
                    last_ok = time.monotonic()
                else:
                    self._closing.wait(0.5)


def serve_mjpeg(handler, get_jpeg, fps=10, stop_on_empty=False):
    """최신 JPEG 를 multipart 로 흘린다.

    같은 프레임이어도 매번 보내고 Content-Length를 붙이지 않는다. 바뀔 때만
    보내면 브라우저가 첫 프레임에서 멈추는 조합이 있어 연결 안정성을 우선한다.
    """
    handler.send_response(200)
    handler.send_header('Cache-Control', 'no-store')
    handler.send_header('Content-Type',
                        'multipart/x-mixed-replace; boundary=frame')
    handler.end_headers()
    try:
        while True:
            j = get_jpeg()
            if j:
                handler.wfile.write(b'--frame\r\n'
                                    b'Content-Type: image/jpeg\r\n\r\n')
                handler.wfile.write(j)
                handler.wfile.write(b'\r\n')
            elif stop_on_empty:
                break
            time.sleep(1.0 / fps)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass                              # 탭을 닫으면 여기로 — 정상 종료


def shutdown_runtime(worker, recorder=None, base_monitor=None, camera=None,
                     mirror=None, *, reason='panel server shutdown', timeout=2.0):
    """ROS 감시를 멈춘 뒤 Worker 명령을 terminal로 만들고 자원을 닫는다."""
    errors = []
    if base_monitor is not None:
        try:
            base_monitor.stop()
            base_monitor.join(timeout)
        except Exception as e:
            errors.append(f'base monitor: {type(e).__name__}: {e}')
    worker_stopped = False
    try:
        worker_stopped = bool(worker.shutdown(reason, timeout))
    except Exception as e:
        errors.append(f'worker: {type(e).__name__}: {e}')
    if not worker_stopped and not any(error.startswith('worker:') for error in errors):
        errors.append('worker: shutdown timeout')
    for name, component in (('recorder', recorder), ('camera', camera),
                            ('mirror', mirror)):
        if component is None:
            continue
        try:
            if name == 'recorder':
                stopped = component.shutdown()
                if stopped is False:
                    errors.append('recorder: shutdown 실패')
            else:
                stopped = component.shutdown(timeout)
                if stopped is False:
                    errors.append(f'{name}: shutdown timeout')
        except Exception as e:
            errors.append(f'{name}: {type(e).__name__}: {e}')
    if errors:
        raise RuntimeError('종료 정리 실패 — ' + ' | '.join(errors))
    return worker_stopped


def make_handler(worker, kin, cam, mir=None, rec=None):
    csrf_token = secrets.token_urlsafe(32)
    page = (HERE / 'panel.html').read_text().replace(
        '"__SO101_CSRF__"', json.dumps(csrf_token)).encode()
    recorded, record_lock = OrderedDict(), threading.Lock()

    def on_terminal(command):
        if (rec is None or command.get('status') != 'completed'
                or not command.get('applied_action')):
            return
        command_id = str(command.get('id') or '')
        if not command_id:
            return
        with record_lock:
            if command_id in recorded:
                return
            recorded[command_id] = None
            while len(recorded) > 256:
                recorded.popitem(last=False)
        rec.note_command(command)

    def terminal_tombstone_count():
        with record_lock:
            return len(recorded)

    remove_terminal_listener = worker.add_terminal_listener(on_terminal)

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):                # 콘솔 잡음 끄기
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _request_host_ok(self):
            expected_port = self.server.server_address[1]
            parsed = _host_port(self.headers.get('Host'), expected_port)
            return parsed is not None and parsed[1] == expected_port

        def _authorize_command(self):
            if not self._request_host_ok():
                self._json({'error': 'forbidden host'}, 403)
                return False
            origin = self.headers.get('Origin')
            if origin:
                port = self.server.server_address[1]
                if not _same_origin(origin, port):
                    self._json({'error': 'forbidden origin'}, 403)
                    return False
                supplied = self.headers.get('X-SO101-CSRF', '').encode()
                if not secrets.compare_digest(supplied, csrf_token.encode()):
                    self._json({'error': 'invalid csrf token'}, 403)
                    return False
            else:
                peer = self.client_address[0]
                if not _loopback(peer):
                    self._json({'error': 'loopback client required'}, 403)
                    return False
            media_type = self.headers.get('Content-Type', '').split(';', 1)[0]
            if media_type.strip().lower() != 'application/json':
                self._json({'error': 'application/json required'}, 415)
                return False
            return True

        def _submit_command(self, op, *args):
            """모든 팔 명령을 추적 가능한 Worker 공개 경계로 제출한다."""
            submit = getattr(worker, 'submit', None)
            status = getattr(worker, 'command_status', None)
            if not callable(submit) or not callable(status):
                raise RuntimeError('Worker 명령 추적 인터페이스가 없습니다')
            command_id = submit(op, *args)
            if not isinstance(command_id, str) or not command_id:
                raise RuntimeError('Worker가 command_id를 반환하지 않았습니다')
            return command_id

        def _command_response(self, command_id, **extra):
            status = worker.command_status(command_id)
            if not isinstance(status, dict):
                raise RuntimeError('명령 상태를 찾을 수 없습니다')
            phase = status.get('status')
            if phase not in ('accepted', 'executing', 'completed', 'rejected'):
                raise RuntimeError('Worker 명령 상태가 유효하지 않습니다')
            body = {
                'ok': phase != 'rejected',
                'command_id': command_id,
                'status': phase,
                'reason': status.get('reason'),
            }
            body.update(extra)
            return self._json(body)

        def do_GET(self):
            if not self._request_host_ok():
                return self._json({'error': 'forbidden host'}, 403)
            parsed = urlsplit(self.path)
            if parsed.path == '/command':
                try:
                    query = parse_qs(parsed.query, strict_parsing=True,
                                     keep_blank_values=True)
                except ValueError:
                    return self._json({'error': 'invalid query'}, 400)
                ids = query.get('id')
                if (set(query) != {'id'} or not ids or len(ids) != 1
                        or not ids[0] or len(ids[0]) > 128):
                    return self._json({'error': 'one valid command id required'}, 400)
                status = worker.command_status(ids[0])
                if not isinstance(status, dict):
                    return self._json({'error': 'command not found'}, 404)
                return self._json(status)
            if parsed.query:
                return self._json({'error': 'query not allowed'}, 400)
            if parsed.path in ('/', '/index.html'):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('Content-Security-Policy',
                                 "default-src 'self'; img-src 'self'; "
                                 "style-src 'self' 'unsafe-inline'; "
                                 "script-src 'self' 'unsafe-inline'; "
                                 "frame-ancestors 'none'; base-uri 'none'")
                self.send_header('Content-Length', str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif parsed.path == '/state':
                s = worker.snapshot()
                # 차량 프로필은 손목캠 단독이다. Astra 상태를 섞지 않아 연결된 장치와
                # 현재 제어 계약이 화면에서 서로 다르게 보이는 일을 막는다.
                s['vision'] = {'source': 'wrist', 'depth': False,
                               'yolo_gate': 'cable_recheck'}
                if cam is None:
                    s['vision']['camera'] = {
                        'sequence': 0, 'captured_at': None, 'age': None, 'stale': True,
                        'available': False}
                else:
                    frame = cam.snapshot_frame()
                    s['vision']['camera'] = {
                        'sequence': frame['sequence'],
                        'captured_at': frame['captured_at'], 'age': frame['age'],
                        'stale': frame['stale'], 'available': True}
                self._json(s)
            elif parsed.path == '/cam':
                if cam is None:
                    return self._json({'error': 'camera off'}, 503)
                cam.ensure()
                if cam.snapshot_frame()['stale']:
                    return self._json({'error': 'camera frame stale'}, 503)
                # MJPEG 스트림 — 브라우저 <img>가 그대로 재생한다
                serve_mjpeg(self, cam.snapshot_jpeg, stop_on_empty=True)
            elif parsed.path == '/frame.jpg':
                if cam is None:
                    return self._json({'error': 'camera off'}, 503)
                cam.ensure()
                frame = cam.snapshot_frame()
                if frame['stale'] or frame['jpeg'] is None:
                    return self._json({'error': 'camera frame stale'}, 503)
                body = frame['jpeg']
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('X-Frame-Sequence', str(frame['sequence']))
                self.send_header('X-Frame-Captured-At', str(frame['captured_at']))
                self.send_header('X-Frame-Age', str(frame['age']))
                self.end_headers()
                self.wfile.write(body)
            elif parsed.path == '/mirror':
                if mir is None:
                    return self._json({'error': 'mirror off'}, 503)
                mir.ensure()
                serve_mjpeg(self, mir.snapshot_jpeg, stop_on_empty=True)
            elif parsed.path == '/mirror/state':
                if mir is None:
                    return self._json({'ok': False, 'msg': 'mirror off'}, 503)
                mir.ensure()
                with mir.lock:
                    st = dict(mir.status)
                self._json({'proxy': st, 'daemon': mir.request('/state')})
            elif parsed.path == '/rec/status':
                if rec is None:
                    return self._json({'recording': False, 'msg': '레코더 비활성'})
                self._json(rec.status())
            elif parsed.path == '/rec/list':
                self._json({'datasets': ds_record.list_datasets(),
                            'root': str(ds_record.DEFAULT_ROOT)})
            else:
                self._json({'error': 'not found'}, 404)

        def do_POST(self):
            # 교차 출처 방어 — 임의 웹페이지의 JS 도 127.0.0.1 로 POST 를 쏠 수
            # 있고(응답만 못 읽을 뿐 명령은 실행된다), /cmd 는 토크를 푼다.
            # Origin 이 붙어 있는데 우리 것이 아니면 거절한다. 로컬 스크립트
            # (urllib 등)는 Origin 을 안 보내므로 영향이 없다.
            if self.path != '/cmd':
                return self._json({'error': 'not found'}, 404)
            if not self._authorize_command():
                return
            try:
                n = int(self.headers.get('Content-Length', ''))
            except ValueError:
                return self._json({'error': 'invalid content length'}, 400)
            if n < 2 or n > MAX_COMMAND_BYTES:
                return self._json({'error': 'command body size rejected'}, 413)
            try:
                req = json.loads(
                    self.rfile.read(n), parse_constant=reject_json_constant)
            except (json.JSONDecodeError, UnicodeDecodeError, ValueError,
                    RecursionError):
                return self._json({'error': 'invalid json'}, 400)
            if not isinstance(req, dict):
                return self._json({'error': 'json object required'}, 400)
            try:
                validate_json_values(req)
            except (ValueError, RecursionError) as e:
                return self._json({'error': str(e)}, 400)
            op = req.get('op')
            try:
                if op == 'stop':
                    command_id = worker.stop_and_cancel('운영자 정지')
                    return self._command_response(command_id)
                elif op in ('connect', 'disconnect', 'neutral', 'save_calib',
                            'rearm'):
                    command_id = self._submit_command(op)
                elif op == 'torque':
                    command_id = self._submit_command('torque', bool(req['on']))
                elif op == 'range':
                    command_id = self._submit_command('range', bool(req['start']))
                elif op == 'jog':
                    command_id = self._submit_command(
                        'jog', req['joint'], float(req['delta']))
                elif op == 'teleop_profile':
                    command_id = self._submit_command(
                        'teleop_profile', bool(req.get('on', True)))
                elif op == 'pose':
                    joints = dict(req.get('joints') or {})
                    command_id = self._submit_command('pose', joints)
                    return self._command_response(command_id)
                elif op == 'goto':
                    command_id = self._submit_command(
                        'goto', req['joint'], float(req['value']))
                elif op == 'stop_test':
                    command_id = self._submit_command(
                        'stop_test', req['joint'], float(req['target']),
                        float(req.get('wait', 1.0)))
                elif op == 'grip_test':
                    command_id = self._submit_command('grip_test', float(req['delta']))
                elif op == 'pan_lock':
                    command_id = self._submit_command(
                        'pan_lock', bool(req.get('on', True)),
                        float(req.get('tol', 0.0)),
                        (float(req['center']) if req.get('center')
                         is not None else None))
                elif op == 'grip_force':
                    command_id = self._submit_command(
                        'grip_force', int(req.get('pct', 45)))
                elif op == 'speed':
                    command_id = self._submit_command('speed', int(req['pct']))
                elif op == 'home':
                    mapping = arm_lib.load_mapping()
                    hq = mapping.get('home_q')
                    if not (isinstance(hq, list) and len(hq) == len(arm_lib.JOINTS)):
                        raise RuntimeError('mapping.json에 차량 홈 자세 home_q가 없습니다')
                    command_id = self._submit_command('move_q', hq, 2.5)
                elif op in ('ik', 'ik_preview'):
                    x, y, z = (float(req[k]) for k in 'xyz')
                    pitch = math.radians(float(req.get('pitch', -90)))
                    bf = tuple(p + o for p, o in zip((x, y, z), arm_lib.PAN0))
                    q = kin.ik_best(*bf, pitch=pitch)
                    if q is None:
                        return self._json({'ok': False,
                                           'msg': 'IK 해 없음 — 리치/한계 밖'})
                    if req.get('roll') is not None:
                        # 손목 롤만 지정각(서보 °)으로 덮어씀 — 누운 물체 방향
                        # 파지용(2026-08-20). 롤은 TCP 축 회전이라 위치 IK 와
                        # 독립이고, 캘리브 범위는 Worker 게이트가 검사한다.
                        mpj = arm_lib.load_mapping()
                        q = list(q)
                        q[4] = math.radians(
                            (float(req['roll']) - mpj['offsets']['wrist_roll'])
                            / mpj['signs']['wrist_roll'])
                    fk = kin.fk_pos(q)
                    pan = [round(p - o, 4) for p, o in zip(fk, arm_lib.PAN0)]
                    mpj = arm_lib.load_mapping()
                    tgt = {j: round(mpj['signs'][j] * math.degrees(q[i])
                                    + mpj['offsets'][j], 2)
                           for i, j in enumerate(arm_lib.JOINTS)}
                    if op == 'ik_preview':
                        # ★ 실행 전 검증 (plan → validate → execute). 팔에는
                        # 아무 명령도 내리지 않는다 — 캘리브 범위를 실제 이동과
                        # 같은 게이트로 미리 보고, 자세를 미러에 띄운다. 거부될
                        # 자세를 팔이 먼저 알게 되는 경로를 없앤다.
                        why, _bad = worker._clamp_to_calib(dict(tgt))
                        cur = worker.snapshot().get('pos') or {}
                        deg = dict(tgt, gripper=cur.get('gripper', 30))
                        shown = False
                        if mir is not None:
                            mir.ensure()
                            shown = bool(mir.request(
                                '/preview',
                                {'deg': deg,
                                 'hold': float(req.get('hold', 4.0))}
                            ).get('ok'))
                        return self._json({'ok': why is None, 'preview': True,
                                           'mirror': shown, 'msg': why or '',
                                           'q': [round(v, 4) for v in q],
                                           'fk_pan': pan, 'deg': tgt})
                    try:
                        secs = float(worker.estimate_motion_duration(tgt))
                    except (AttributeError, RuntimeError, TypeError, ValueError) as e:
                        raise RuntimeError(f'이동 시간 안전 계산 실패: {e}') from e
                    if not math.isfinite(secs) or secs <= 0:
                        raise RuntimeError('이동 시간 안전 계산이 유효한 값을 주지 않았습니다')
                    command_id = self._submit_command(
                        'move_q', list(q), round(secs, 2))
                    return self._command_response(
                        command_id, q=[round(v, 4) for v in q], fk_pan=pan)
                elif op in ('rec_start', 'rec_stop', 'rec_cancel', 'rec_replay'):
                    if rec is None:
                        return self._json({'ok': False, 'msg': '레코더 비활성'}, 503)
                    if op == 'rec_start':
                        rid = str(req.get('repo_id') or '').strip()
                        if not valid_repo_id(rid):
                            return self._json({'ok': False,
                                               'msg': '데이터셋 이름은 영문·숫자·_·- '
                                                      '1~64자만 허용됩니다'}, 400)
                        if req.get('wrist', True) is not True:
                            return self._json(
                                {'ok': False, 'msg': '차량 기록은 손목캠이 필수입니다'},
                                503)
                        ready, evidence = record_start_readiness(worker, cam)
                        if not ready:
                            return self._json({'ok': False, 'msg': evidence}, 503)
                        result = rec.start_episode(
                            rid, req.get('task') or '', int(req.get('fps', 10)),
                            wrist=bool(req.get('wrist', True)),
                            depth=False, pointmap=False,
                            capability=evidence,
                            validate_capability=make_record_capability_validator(
                                worker, cam))
                        return self._json(result, 200 if result.get('ok') else 503)
                    if op == 'rec_stop':
                        result = rec.stop_episode(save=True)
                        return self._json(result, 200 if result.get('ok') else 503)
                    if op == 'rec_cancel':
                        result = rec.stop_episode(save=False)
                        return self._json(result, 200 if result.get('ok') else 503)
                    # rec_replay — 기록된 궤적을 미러에서 되돌려 본다(팔 정지)
                    rid = str(req.get('repo_id') or '').strip()
                    if not valid_repo_id(rid):
                        return self._json({'ok': False,
                                           'msg': '잘못된 데이터셋 이름'}, 400)
                    if mir is None:
                        return self._json({'ok': False, 'msg': '미러 비활성'}, 503)
                    try:
                        frames = ds_record.episode_frames(
                            rid, int(req.get('episode', 0)),
                            stride=int(req.get('stride', 1)))
                    except Exception as e:
                        return self._json({'ok': False,
                                           'msg': f'에피소드 읽기 실패: {e}'}, 400)
                    if not frames:
                        return self._json({'ok': False, 'msg': '프레임 없음'}, 400)
                    mir.ensure()
                    r = mir.request('/replay', {'frames': frames,
                                                'fps': float(req.get('fps', 10))},
                                    timeout=20.0)
                    return self._json(dict(r, frames=len(frames)))
                elif op in ('mirror_view', 'mirror_live', 'mirror_replay',
                            'mirror_piece'):
                    # 미러는 표시 계층이라 팔을 건드리지 않는다 — 데몬으로 중계만
                    if mir is None:
                        return self._json({'ok': False, 'msg': '미러 비활성'}, 503)
                    mir.ensure()
                    if op == 'mirror_view':
                        body = {k: req[k] for k in
                                ('azimuth', 'elevation', 'distance') if k in req}
                        return self._json(mir.request('/view', body))
                    if op == 'mirror_live':
                        return self._json(mir.request('/live', {}))
                    if op == 'mirror_piece':
                        body = {k: req[k] for k in ('x', 'y', 'yaw') if k in req}
                        return self._json(mir.request('/piece', body))
                    return self._json(mir.request(
                        '/replay', {'frames': req.get('frames') or [],
                                    'fps': float(req.get('fps', 10))},
                        timeout=10.0))
                else:
                    return self._json({'error': f'unknown op {op}'}, 400)
                self._command_response(command_id)
            except RuntimeError as e:
                self._json({'ok': False, 'msg': str(e)}, 503)
            except (KeyError, TypeError, ValueError, OverflowError) as e:
                self._json({'ok': False, 'msg': f'인자 오류: {e}'}, 400)

    H.remove_terminal_listener = staticmethod(remove_terminal_listener)
    H.terminal_tombstone_count = staticmethod(terminal_tombstone_count)
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port-serial', default='auto',
                    help="시리얼 포트. 'auto'면 /dev/ttyACM* 중 첫 번째를 잡는다 "
                         "— USB를 다시 꽂으면 번호가 바뀌므로(ACM1→ACM0 실측) "
                         "고정 경로로 띄운 서버는 조용히 죽은 포트를 붙든다")
    ap.add_argument('--id', default='follower')
    ap.add_argument('--http', type=int, default=8765)
    ap.add_argument('--cam', type=int, default=4,
                    help='V4L2 인덱스 (/dev/videoN). -1이면 캠 끔')
    ap.add_argument('--no-mirror', dest='mirror', action='store_false',
                    help='MuJoCo 미러(디지털 트윈)를 끈다')
    ap.add_argument('--mirror-port', type=int, default=8768,
                    help='미러 렌더 데몬(sim/mirror_daemon.py)의 HTTP 포트')
    ap.add_argument('--piece', default='cube',
                    choices=['cube', 'lying', 'standing'],
                    help='미러에 그릴 물체 프록시')
    ap.add_argument('--no-record', dest='record', action='store_false',
                    help='LeRobot 데이터셋 기록 기능을 끈다')
    a = ap.parse_args()

    port = a.port_serial
    if port == 'auto':
        # 신원 검증을 통과한 포트만 (arm_lib.find_arm_port) — 팔이 꺼져 있을 때
        # 남의 USB-시리얼 보드를 팔로 집는 사고를 막는다 (2026-08-21 실측)
        cands = [p for p in [arm_lib.find_arm_port()] if p]
        if not cands:
            # 포트가 없어도 서버는 띄운다 (2026-08-21). 여기서 죽으면 팔 전원이
            # 꺼져 있다는 이유로 미러·손목캠·패널까지 통째로 못 뜬다 — 데이터셋
            # 검수나 시뮬 작업은 팔 없이도 하는 일이다. Worker._do_connect 가
            # 연결 시점에 포트를 재탐색하므로, 나중에 꽂아도 그대로 붙는다.
            port = '/dev/ttyACM0'
            print('팔로 확인된 시리얼 포트 없음 — 팔 없이 기동합니다 '
                  '(전원·USB 를 연결하고 [연결]을 누르면 포트를 다시 찾습니다)')
        else:
            port = cands[0]
            print(f'포트 자동 선택: {port}')

    worker = Worker(port, a.id)
    worker.start()
    base_monitor = BaseMonitor(worker)
    base_monitor.start()
    kin = arm_lib.load_kinematics()
    if a.cam >= 0:
        idx = Camera.find_index(a.cam if a.cam != 4 else None)
        if idx is None:
            cam = None
            print('손목캠 미검출(외장 UVC 없음) — /cam 비활성. USB 허브 연결·전원 확인')
        else:
            cam = Camera(idx)
            print(f'카메라 자동 선택: /dev/video{idx}')
    else:
        cam = None

    mir = None
    if a.mirror:
        if Mirror.python_bin() is None:
            print('mujoco 환경(rlwalk) 없음 — 미러 비활성 (/mirror 503)')
        else:
            mir = Mirror(a.mirror_port, a.piece)   # 첫 요청에 기동(ensure)
    rec = ds_record.Recorder(worker, cam, None) if a.record else None
    ThreadingHTTPServer.daemon_threads = True   # 남은 스트림 스레드가 종료를 막지 않게
    # ★ 단일 인스턴스 잠금 (2026-08-24) — 패널이 둘 뜨면 같은 시리얼 포트를
    # 두 프로세스가 물고 명령 유실·패킷 실패·유령 무응답이 생긴다(하루 종일
    # 실측). flock 이라 프로세스가 어떻게 죽든 잠금은 자동 해제된다.
    import fcntl
    global _panel_lock
    _panel_lock = open('/tmp/so101_panel.lock', 'w')
    try:
        fcntl.flock(_panel_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print('다른 패널 인스턴스가 이미 실행 중입니다 — 종료 (중복 기동 금지)')
        sys.exit(1)
    _panel_lock.write(str(os.getpid())); _panel_lock.flush()

    handler = make_handler(worker, kin, cam, mir, rec)
    srv = ThreadingHTTPServer(('127.0.0.1', a.http), handler)
    print(f'SO-101 패널 → http://127.0.0.1:{a.http}  (시리얼 {port})')

    # SIGTERM(systemctl stop · kill)에도 정리 경로를 타게 한다. 기본 동작은 즉시
    # 종료 신호에서도 레코더·미러·팔 워커 정리 경로를 반드시 탄다.
    import signal

    def _bye(signum, frame):
        raise KeyboardInterrupt

    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, _bye)

    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            shutdown_runtime(worker, rec, base_monitor, cam, mir,
                             reason='panel server shutdown', timeout=2.0)
        finally:
            handler.remove_terminal_listener()
            srv.server_close()


if __name__ == '__main__':
    main()
