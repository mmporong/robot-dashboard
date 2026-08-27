#!/usr/bin/env python3
"""SO-101 실기 제어 패널 서버 — 대시보드 디자인 언어를 쓰는 라이브 패널.

다른 프로젝트(capstone-pick·slam)의 대시보드는 **기록을 보는** 정적 페이지지만,
이 패널은 **실물 팔을 움직이는** 라이브 페이지라 뒤에 서버가 필요하다. 시리얼
통신은 `~/so101-mobile-manipulation/arm_gui.py`의 `Worker`(전담 스레드)를 쓰고, 이
서버는 그 앞에 HTTP 만 얹는다.

    GET  /        → panel.html
    GET  /state   → Worker 상태 JSON (연결·캘리브·토크·관절각·범위·로그)
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
import json
import math
import pathlib
import subprocess
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = pathlib.Path(__file__).parent
TOOLS = pathlib.Path('~/so101-mobile-manipulation').expanduser()
sys.path.insert(0, str(TOOLS))

import arm_lib                                    # noqa: E402
import ds_record                                  # noqa: E402  (LeRobot 표준 기록)
from arm_gui import Worker                        # noqa: E402  (시리얼 워커 재사용)

HOME_Q = [0.0, -0.3, 0.6, 0.5, 0.0]


class Camera(threading.Thread):
    """UVC 카메라를 10fps로 읽어 최신 JPEG 한 장만 유지한다.

    클라이언트 수와 무관하게 캡처는 한 스레드가 하고, /cam 스트림들은 이 버퍼를
    나눠 읽는다. 첫 요청이 올 때까지 카메라를 열지 않는다(게으른 시작) — 패널만
    띄우고 캠을 안 볼 때 USB 대역·CPU를 안 쓰기 위해서다.
    """

    def __init__(self, index):
        super().__init__(daemon=True)
        self.index = index
        self.lock = threading.Lock()
        self.jpeg = None
        self.started = False
        self._start_lock = threading.Lock()   # 핸들러가 병렬이라 check-then-act 보호

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
        with self.lock:
            return self.jpeg

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
        fails = 0
        while True:
            ok, frame = cap.read()
            if ok:
                fails = 0
                ok2, buf = cv2.imencode('.jpg', frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok2:
                    with self.lock:
                        self.jpeg = buf.tobytes()
            else:
                # 케이블이 빠지면 read 가 영원히 False 다 — 방치하면 /cam 이 마지막
                # JPEG 로 굳는다(깊이 쪽에서 없앤 바로 그 증상). 닫고 다시 연다.
                fails += 1
                if fails >= 50:
                    cap.release()
                    time.sleep(2.0)
                    cap = self._open(cv2)
                    if cap is None:
                        return
                    fails = 0
            time.sleep(0.1)                      # 10fps — 패널 확인용이라 충분


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
        self._closing = False
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
            return self.jpeg

    def ensure(self):
        with self._start_lock:
            if not self.started:
                self.started = True
                self.start()

    def shutdown(self, timeout=6.0):
        self._closing = True
        if self.is_alive():
            self.join(2.0)
        self._stop_proc(grace=timeout)

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

    def _spawn(self):
        with self._proc_lock:
            if self._closing:
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
                time.sleep(1.5)
            if self._log is None:
                self._log = open(HERE / 'mirror_daemon.log', 'ab', buffering=0)
            self._proc = subprocess.Popen(
                [py, '-u', str(self.SIM_DIR / 'mirror_daemon.py'),
                 '--http', str(self.port), '--piece', self.piece],
                cwd=str(self.SIM_DIR), stdout=self._log,
                stderr=subprocess.STDOUT)

    def _stop_proc(self, grace=6.0):
        with self._proc_lock:
            p, self._proc = self._proc, None
        if p is None or p.poll() is not None:
            return
        try:
            p.terminate()
            p.wait(grace)
        except Exception:
            try:
                p.kill()
                p.wait(3.0)
            except Exception:
                pass

    def run(self):
        import urllib.request
        self._spawn()
        last_ok = time.monotonic()
        while not self._closing:
            time.sleep(0.1)
            try:
                url = f'http://127.0.0.1:{self.port}/frame.jpg'
                with urllib.request.urlopen(url, timeout=3.0) as r:
                    j = r.read()
                if j:
                    with self.lock:
                        self.jpeg = j
                        self.status = {'ok': True, 'msg': ''}
                    last_ok = time.monotonic()
                    self._restarts = 0
            except Exception as e:
                if time.monotonic() - last_ok > 15.0:
                    with self.lock:
                        self.status = {'ok': False,
                                       'msg': f'미러 데몬 무응답 — 재시작 ({type(e).__name__})'}
                    self._stop_proc()
                    self._restarts += 1
                    time.sleep(min(3.0 * (2 ** min(self._restarts - 1, 3)), 30.0))
                    if self._closing:
                        break
                    self._spawn()
                    last_ok = time.monotonic()
                else:
                    time.sleep(0.5)


def serve_mjpeg(handler, get_jpeg, fps=10):
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
            time.sleep(1.0 / fps)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass                              # 탭을 닫으면 여기로 — 정상 종료


def make_handler(worker, kin, cam, mir=None, rec=None):
    page = (HERE / 'panel.html').read_bytes()

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

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif self.path == '/state':
                s = worker.snapshot()
                # 차량 프로필은 손목캠 단독이다. Astra 상태를 섞지 않아 연결된 장치와
                # 현재 제어 계약이 화면에서 서로 다르게 보이는 일을 막는다.
                s['vision'] = {'source': 'wrist', 'depth': False,
                               'yolo_gate': 'cable_recheck'}
                self._json(s)
            elif self.path == '/cam':
                if cam is None:
                    return self._json({'error': 'camera off'}, 503)
                cam.ensure()
                # MJPEG 스트림 — 브라우저 <img>가 그대로 재생한다
                serve_mjpeg(self, cam.snapshot_jpeg)
            elif self.path == '/mirror':
                if mir is None:
                    return self._json({'error': 'mirror off'}, 503)
                mir.ensure()
                serve_mjpeg(self, mir.snapshot_jpeg)
            elif self.path == '/mirror/state':
                if mir is None:
                    return self._json({'ok': False, 'msg': 'mirror off'}, 503)
                mir.ensure()
                with mir.lock:
                    st = dict(mir.status)
                self._json({'proxy': st, 'daemon': mir.request('/state')})
            elif self.path == '/rec/status':
                if rec is None:
                    return self._json({'recording': False, 'msg': '레코더 비활성'})
                self._json(rec.status())
            elif self.path == '/rec/list':
                self._json({'datasets': ds_record.list_datasets(),
                            'root': str(ds_record.DEFAULT_ROOT)})
            else:
                self._json({'error': 'not found'}, 404)

        def do_POST(self):
            # 교차 출처 방어 — 임의 웹페이지의 JS 도 127.0.0.1 로 POST 를 쏠 수
            # 있고(응답만 못 읽을 뿐 명령은 실행된다), /cmd 는 토크를 푼다.
            # Origin 이 붙어 있는데 우리 것이 아니면 거절한다. 로컬 스크립트
            # (urllib 등)는 Origin 을 안 보내므로 영향이 없다.
            origin = self.headers.get('Origin')
            if origin and not origin.startswith(('http://127.0.0.1',
                                                 'http://localhost')):
                return self._json({'error': 'forbidden origin'}, 403)
            if self.path != '/cmd':
                return self._json({'error': 'not found'}, 404)
            n = int(self.headers.get('Content-Length', 0))
            req = json.loads(self.rfile.read(n) or '{}')
            op = req.get('op')
            try:
                if op == 'stop':
                    # 큐를 우회한다 — 보간 이동 중이면 다음 스텝에서 즉시 끊긴다.
                    worker.abort.set()
                    try:                          # 밀려 있는 명령도 전부 버린다
                        while True:
                            worker.cmd.get_nowait()
                    except Exception:
                        pass
                    worker.cmd.put(('stop',))
                elif op in ('connect', 'disconnect', 'neutral', 'save_calib'):
                    worker.cmd.put((op,))
                elif op == 'torque':
                    worker.cmd.put(('torque', bool(req['on'])))
                elif op == 'range':
                    worker.cmd.put(('range', bool(req['start'])))
                elif op == 'jog':
                    worker.cmd.put(('jog', req['joint'], float(req['delta'])))
                    if rec is not None:      # 액션 = 명령 목표 (데이터셋 기록용)
                        cur = (worker.snapshot().get('pos') or {}).get(req['joint'])
                        if cur is not None:
                            rec.note_action({req['joint']:
                                             cur + float(req['delta'])})
                elif op == 'teleop_profile':
                    worker.cmd.put(('teleop_profile', bool(req.get('on', True))))
                    return self._json({'ok': True})
                elif op == 'pose':
                    joints = dict(req.get('joints') or {})
                    worker.cmd.put(('pose', joints))
                    if rec is not None and joints:
                        rec.note_action({j: float(v) for j, v in joints.items()})
                    return self._json({'ok': True})
                elif op == 'goto':
                    worker.cmd.put(('goto', req['joint'], float(req['value'])))
                    if rec is not None:
                        rec.note_action({req['joint']: float(req['value'])})
                elif op == 'stop_test':
                    worker.cmd.put(('stop_test', req['joint'], float(req['target']),
                                    float(req.get('wait', 1.0))))
                elif op == 'grip_test':
                    worker.cmd.put(('grip_test', float(req['delta'])))
                elif op == 'pan_lock':
                    worker.cmd.put(('pan_lock', bool(req.get('on', True)),
                                    float(req.get('tol', 0.0)),
                                    (float(req['center']) if req.get('center')
                                     is not None else None)))
                elif op == 'grip_force':
                    worker.cmd.put(('grip_force', int(req.get('pct', 45))))
                elif op == 'speed':
                    worker.cmd.put(('speed', int(req['pct'])))
                elif op == 'home':
                    # 사용자가 잡아 둔 홈이 mapping.json 에 있으면 그것을 쓴다
                    hq = arm_lib.load_mapping().get('home_q', HOME_Q)
                    worker.cmd.put(('move_q', hq, 2.5))
                    if rec is not None:
                        mph = arm_lib.load_mapping()
                        rec.note_action({j: mph['signs'][j] * math.degrees(hq[i])
                                         + mph['offsets'][j]
                                         for i, j in enumerate(arm_lib.JOINTS)})
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
                    # 보간 시간을 거리 비례로 (2026-08-20): 고정 3초는 짧은
                    # 구간을 굼뜨게, 긴 구간은 속도상한에 눌려 프로파일이
                    # 어긋났다. 최대 관절 이동량 / (상한 × 0.85) 로 잡는다.
                    secs = 3.0
                    try:
                        cur = worker.snapshot().get('pos') or {}
                        md = max(abs((tgt[j] - cur.get(j, tgt[j]) + 180)
                                     % 360 - 180) for j in arm_lib.JOINTS)
                        vel = worker._profile_vel() * 0.087   # [°/s]
                        secs = min(14.0, max(0.8, md / (vel * 0.6)))
                        # 상한 6→14초 (2026-08-26): 차량에서 작업↔관찰 자세는
                        # wrist_flex 가 69° 를 움직인다. 6초 상한이면 11.5°/s 로
                        # 명령이 나가는데 중력을 드는 손목이 못 따라가 스톨 오판.
                    except Exception:
                        pass
                    worker.cmd.put(('move_q', list(q), round(secs, 2)))
                    if rec is not None:
                        rec.note_action(tgt)
                    return self._json({'ok': True,
                                       'q': [round(v, 4) for v in q],
                                       'fk_pan': pan})
                elif op in ('rec_start', 'rec_stop', 'rec_cancel', 'rec_replay'):
                    if rec is None:
                        return self._json({'ok': False, 'msg': '레코더 비활성'}, 503)
                    if op == 'rec_start':
                        rid = str(req.get('repo_id') or '').strip()
                        if not rid or '/' in rid or rid.startswith('.'):
                            return self._json({'ok': False,
                                               'msg': '데이터셋 이름을 확인하세요 '
                                                      '(빈 값·/·. 로 시작 불가)'}, 400)
                        return self._json(rec.start_episode(
                            rid, req.get('task') or '', int(req.get('fps', 10)),
                            wrist=bool(req.get('wrist', True)),
                            depth=False, pointmap=False))
                    if op == 'rec_stop':
                        return self._json(rec.stop_episode(save=True))
                    if op == 'rec_cancel':
                        return self._json(rec.stop_episode(save=False))
                    # rec_replay — 기록된 궤적을 미러에서 되돌려 본다(팔 정지)
                    if mir is None:
                        return self._json({'ok': False, 'msg': '미러 비활성'}, 503)
                    try:
                        frames = ds_record.episode_frames(
                            req['repo_id'], int(req.get('episode', 0)),
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
                self._json({'ok': True})
            except (KeyError, ValueError) as e:
                self._json({'ok': False, 'msg': f'인자 오류: {e}'}, 400)

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

    srv = ThreadingHTTPServer(('127.0.0.1', a.http),
                              make_handler(worker, kin, cam, mir, rec))
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
        if rec is not None:
            rec.shutdown()          # 기록 중이면 저장하고 끝낸다 — 버리지 않는다
        if mir is not None:
            mir.shutdown()          # 렌더 데몬을 남기면 GPU 를 문 고아가 된다
        # ★ 토크 유지 종료 (2026-08-24) — 팔 자세와 무관하게 토크를 끊던 옛
        # 경로는 펴진 팔을 책상에 떨어뜨렸다. 토크 해제는 사용자의 명시적
        # '해제'(disconnect op)에서만.
        worker.cmd.put(('disconnect_hold',))
        worker.stop()


if __name__ == '__main__':
    main()
