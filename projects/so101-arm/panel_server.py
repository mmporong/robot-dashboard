#!/usr/bin/env python3
"""SO-101 실기 제어 패널 서버 — 대시보드 디자인 언어를 쓰는 라이브 패널.

다른 프로젝트(capstone-pick·slam)의 대시보드는 **기록을 보는** 정적 페이지지만,
이 패널은 **실물 팔을 움직이는** 라이브 페이지라 뒤에 서버가 필요하다. 시리얼
통신은 `~/so101_tools/arm_gui.py` 의 `Worker`(전담 스레드)를 그대로 쓰고, 이
서버는 그 앞에 HTTP 만 얹는다.

    GET  /        → panel.html
    GET  /state   → Worker 상태 JSON (연결·캘리브·토크·관절각·범위·로그)
    POST /cmd     → {"op": "connect" | "disconnect" | "torque" | "neutral"
                     | "range" | "save_calib" | "jog" | "ik" | "home"} + 인자

IK 는 서버에서 푼다 — 캡스톤 `kinematics.ik_best` 로 관절각을 만들어 Worker 에
넘긴다. 좌표는 pan 축 기준(x=전방·y=좌·z=상, 원점=베이스 서보 회전 중심).

사용:
    conda activate lerobot
    python3 panel_server.py            # http://127.0.0.1:8765
    python3 panel_server.py --port-serial /dev/ttyACM1 --http 8766
"""
import argparse
import base64
import json
import math
import pathlib
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = pathlib.Path(__file__).parent
TOOLS = pathlib.Path('~/so101_tools').expanduser()
sys.path.insert(0, str(TOOLS))

import arm_lib                                    # noqa: E402
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

    def ensure(self):
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
        pick = ext or cands
        if prefer is not None and any(c[1] == prefer for c in pick):
            return prefer
        return pick[0][1] if pick else 0

    def snapshot_jpeg(self):
        with self.lock:
            return self.jpeg

    def run(self):
        import cv2
        cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():                    # 번호가 밀렸으면 다시 찾는다
            alt = self.find_index()
            if alt != self.index:
                self.index = alt
                cap = cv2.VideoCapture(alt)
        if not cap.isOpened():
            return
        # ★ MJPG 강제 + 15fps. 기본 포맷(YUYV 640x480@30)은 비압축이라 등시성
        # 대역 ~147Mbps 를 예약하는데, 이 캠은 Astra 와 같은 USB2(480M) 버스에
        # 있다. 그 예약이 걸리는 순간 Astra 프레임이 굶고, 구형 SDK 는 거기서
        # 영구 교착한다(실측 2026-08-19: /cam 첫 접속에 깊이 스트림 동결).
        # MJPG 는 수십 Mbps 라 둘이 공존한다. 어차피 /cam 은 10fps 로만 내보낸다.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 15)
        while True:
            ok, frame = cap.read()
            if ok:
                ok2, buf = cv2.imencode('.jpg', frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok2:
                    with self.lock:
                        self.jpeg = buf.tobytes()
            time.sleep(0.1)                      # 10fps — 패널 확인용이라 충분


class Depth(threading.Thread):
    """깊이 데몬(depth_daemon.py)을 감독하고 최신 프레임을 중계한다.

    손목캠과 달리 이쪽은 **거리를 직접 측정**한다. 손목캠이 못 푸는 전후(x) 를
    여기서 얻는 것이 이 카메라를 붙인 이유다(2026-08-18: 면적 기반 x 추정이 두 번
    실패 — 신호가 거리와 무관하게 움직였다).

    ★ 캡처는 같은 프로세스가 아니라 **별도 프로세스**가 한다. Legacy SDK 는
    같은 USB2 버스의 UVC 캠이 열리는 순간 astra_update() 안에서 영구 교착할 수
    있는데(실측 2026-08-19: /cam 최초 접속에 깊이 스레드가 C 코드에서 멈춰
    재연결 루프조차 안 돌았다), 교착한 스레드는 살릴 방법이 없고 서버 재시작은
    팔 토크를 풀어 버린다. 프로세스로 떼어 두면 데몬만 갈아끼우면 된다.

    이 스레드가 하는 일: 데몬 기동 → /all 을 10Hz 로 끌어와 캐시 → 하트비트가
    멎거나 프로세스가 죽으면 재기동. 밖에서 보는 인터페이스(lock·stats·blob·
    snapshot_jpeg·ensure·shutdown)는 종전 그대로다.
    """

    # 데몬 HTTP 가 살아 있어도 하트비트(beat_age)가 이보다 오래 멎으면 캡처가
    # SDK 안에서 굳은 것이다 — 데몬 자체 워치독(12s)이 놓친 경우의 안전망.
    STALE_S = 20.0
    # /all 요청이 이 시간 동안 계속 실패하면(기동 직후 제외) 데몬을 갈아끼운다.
    HTTP_DEAD_S = 15.0

    def __init__(self, port=8766):
        super().__init__(daemon=True)
        self.port = port
        self.lock = threading.Lock()
        self.jpeg = None
        self.rgb_jpeg = None
        self.stats = {'ok': False, 'msg': '시작 전'}
        self.blob = None
        self.started = False
        self._closing = False
        self._proc = None
        self._log = None

    def snapshot_jpeg(self, attr):
        with self.lock:
            return getattr(self, attr)

    def ensure(self):
        if not self.started:
            self.started = True
            self.start()

    def shutdown(self, timeout=10.0):
        """데몬을 곱게 끝낸다 — SIGTERM 이면 데몬이 Astra 를 스스로 닫는다."""
        self._closing = True
        if self.is_alive():
            self.join(2.0)
        self._stop_proc(grace=timeout)

    def _spawn(self):
        if self._log is None:
            self._log = open(HERE / 'depth_daemon.log', 'ab', buffering=0)
        self._proc = subprocess.Popen(
            [sys.executable, '-u', str(HERE / 'depth_daemon.py'),
             '--http', str(self.port)],
            cwd=str(HERE), stdout=self._log, stderr=subprocess.STDOUT)

    def _stop_proc(self, grace=10.0):
        """SIGTERM → 대기 → SIGKILL. grace 를 넉넉히 — 데몬이 Astra 를 닫는 데
        시간이 걸리고, 안 닫힌 채 죽으면 다음 열기가 한동안 실패한다."""
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
        last_seq = -1
        now = time.monotonic()
        last_http_ok = last_beat_ok = now
        while not self._closing:
            time.sleep(0.1)
            now = time.monotonic()
            # ① 데몬이 스스로 죽었나 (자체 워치독의 SDK 교착 감지 등)
            if self._proc is not None and self._proc.poll() is not None:
                code = self._proc.returncode
                with self.lock:
                    self.stats = {'ok': False, 'msg': f'데몬 재시작 중 (exit {code})'}
                self._proc = None
                time.sleep(3.0)               # 커널이 usbfs 를 회수할 시간
                if self._closing:
                    break
                self._spawn()
                last_http_ok = last_beat_ok = time.monotonic()
                continue
            # ② 프레임·상태 끌어오기
            try:
                with urllib.request.urlopen(
                        f'http://127.0.0.1:{self.port}/all', timeout=0.8) as r:
                    d = json.loads(r.read())
            except Exception:
                d = None
            if d is not None:
                last_http_ok = now
                if d.get('beat_age', 0) <= self.STALE_S:
                    last_beat_ok = now
                if d.get('seq', -1) != last_seq:
                    last_seq = d['seq']
                    with self.lock:
                        if d.get('depth_jpeg'):
                            self.jpeg = base64.b64decode(d['depth_jpeg'])
                        if d.get('rgb_jpeg'):
                            self.rgb_jpeg = base64.b64decode(d['rgb_jpeg'])
                        self.stats = d.get('stats') or self.stats
                        self.blob = d.get('blob')
                else:                          # 프레임은 그대로여도 상태는 싣는다
                    with self.lock:
                        self.stats = d.get('stats') or self.stats
            # ③ 데몬이 살아는 있는데 응답이 없거나 캡처가 굳음 → 갈아끼운다
            if (now - last_http_ok > self.HTTP_DEAD_S
                    or now - last_beat_ok > self.STALE_S + 5):
                with self.lock:
                    self.stats = {'ok': False, 'msg': '데몬 응답 없음 — 재시작'}
                self._stop_proc()
                time.sleep(3.0)
                if self._closing:
                    break
                self._spawn()
                last_http_ok = last_beat_ok = time.monotonic()


def serve_mjpeg(handler, get_jpeg, fps=10):
    """최신 JPEG 를 multipart 로 흘린다.

    ★ 같은 프레임이어도 매번 보낸다. "바뀔 때만 보내기"로 최적화했다가 브라우저가
    첫 프레임만 그리고 멈췄다(2026-08-19 실측: /cam 은 정상인데 /depth 만 정지,
    <img> 가 새 파트를 못 받는 상태). multipart/x-mixed-replace 는 파트가 꾸준히
    와야 <img> 가 갱신되므로, 대역을 아끼려 들지 않는다. Content-Length 도 붙이지
    않는다 — 넣으면 브라우저가 첫 파트만 읽고 끝낸다.
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


def make_handler(worker, kin, cam, dep):
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
                if dep is not None:
                    with dep.lock:
                        s['depth'] = dict(dep.stats)
                self._json(s)
            elif self.path == '/blob':
                # 정합용 측정창 — 뎁스캠이 본 빨간 물체의 카메라 좌표를 그대로 준다.
                if dep is None:
                    return self._json({'ok': False, 'msg': 'depth off'}, 503)
                dep.ensure()
                with dep.lock:
                    b, st = dep.blob, dict(dep.stats)
                self._json({'ok': b is not None, 'blob': b, 'depth': st})
            elif self.path == '/rgb':
                if dep is None:
                    return self._json({'error': 'depth off'}, 503)
                dep.ensure()
                serve_mjpeg(self, lambda: dep.snapshot_jpeg('rgb_jpeg'))
            elif self.path == '/depth':
                if dep is None:
                    return self._json({'error': 'depth off'}, 503)
                dep.ensure()
                serve_mjpeg(self, lambda: dep.snapshot_jpeg('jpeg'))
            elif self.path == '/cam':
                if cam is None:
                    return self._json({'error': 'camera off'}, 503)
                cam.ensure()
                # MJPEG 스트림 — 브라우저 <img>가 그대로 재생한다
                serve_mjpeg(self, cam.snapshot_jpeg)
            else:
                self._json({'error': 'not found'}, 404)

        def do_POST(self):
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
                elif op == 'goto':
                    worker.cmd.put(('goto', req['joint'], float(req['value'])))
                elif op == 'stop_test':
                    worker.cmd.put(('stop_test', req['joint'], float(req['target']),
                                    float(req.get('wait', 1.0))))
                elif op == 'grip_test':
                    worker.cmd.put(('grip_test', float(req['delta'])))
                elif op == 'speed':
                    worker.cmd.put(('speed', int(req['pct'])))
                elif op == 'home':
                    # 사용자가 잡아 둔 홈이 mapping.json 에 있으면 그것을 쓴다
                    hq = arm_lib.load_mapping().get('home_q', HOME_Q)
                    worker.cmd.put(('move_q', hq, 2.5))
                elif op == 'ik':
                    x, y, z = (float(req[k]) for k in 'xyz')
                    pitch = math.radians(float(req.get('pitch', -90)))
                    bf = tuple(p + o for p, o in zip((x, y, z), arm_lib.PAN0))
                    q = kin.ik_best(*bf, pitch=pitch)
                    if q is None:
                        return self._json({'ok': False,
                                           'msg': 'IK 해 없음 — 리치/한계 밖'})
                    fk = kin.fk_pos(q)
                    pan = [round(p - o, 4) for p, o in zip(fk, arm_lib.PAN0)]
                    worker.cmd.put(('move_q', list(q), 3.0))
                    return self._json({'ok': True,
                                       'q': [round(v, 4) for v in q],
                                       'fk_pan': pan})
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
    ap.add_argument('--no-depth', dest='depth', action='store_false',
                    help='Orbbec 깊이 스트림을 끈다')
    ap.add_argument('--depth-port', type=int, default=8766,
                    help='깊이 캡처 데몬(depth_daemon.py)의 HTTP 포트')
    ap.add_argument('--cam', type=int, default=4,
                    help='V4L2 인덱스 (/dev/videoN). -1이면 캠 끔')
    a = ap.parse_args()

    port = a.port_serial
    if port == 'auto':
        import glob
        cands = sorted(glob.glob('/dev/ttyACM*')) or sorted(glob.glob('/dev/ttyUSB*'))
        if not cands:
            raise SystemExit('시리얼 포트를 못 찾았어요 — USB 케이블과 보드 전원을 확인하세요')
        port = cands[0]
        print(f'포트 자동 선택: {port}')

    worker = Worker(port, a.id)
    worker.start()
    kin = arm_lib.load_kinematics()
    if a.cam >= 0:
        idx = Camera.find_index(a.cam if a.cam != 4 else None)
        cam = Camera(idx)
        print(f'카메라 자동 선택: /dev/video{idx}')
    else:
        cam = None

    dep = Depth(a.depth_port) if a.depth else None
    ThreadingHTTPServer.daemon_threads = True   # 남은 스트림 스레드가 종료를 막지 않게
    srv = ThreadingHTTPServer(('127.0.0.1', a.http),
                              make_handler(worker, kin, cam, dep))
    print(f'SO-101 패널 → http://127.0.0.1:{a.http}  (시리얼 {port})')

    # SIGTERM(systemctl stop · kill)에도 정리 경로를 타게 한다. 기본 동작은 즉시
    # 종료라 finally 가 실행되지 않고, 그러면 Astra 를 열어 둔 채 프로세스만
    # 사라져 다음 기동이 실패한다.
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
        if dep is not None:
            dep.shutdown()
        worker.cmd.put(('disconnect',))
        worker.stop()


if __name__ == '__main__':
    main()
