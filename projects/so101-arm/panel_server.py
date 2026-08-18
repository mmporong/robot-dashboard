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
import json
import math
import pathlib
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
    """Orbbec Astra 깊이를 컬러맵 JPEG 로 만들어 최신 한 장만 유지한다.

    손목캠과 달리 이쪽은 **거리를 직접 측정**한다. 손목캠이 못 푸는 전후(x) 를
    여기서 얻는 것이 이 카메라를 붙인 이유다(2026-08-18: 면적 기반 x 추정이 두 번
    실패 — 신호가 거리와 무관하게 움직였다).

    SDK 가 구형이라 별도 프로세스로 띄우지 않고 같은 프로세스에서 ctypes 로 연다.
    실패해도 패널 전체가 죽지 않도록 예외를 삼키고 상태만 남긴다.
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.jpeg = None
        self.stats = {'ok': False, 'msg': '시작 전'}
        self.started = False

    def ensure(self):
        if not self.started:
            self.started = True
            self.start()

    def run(self):
        import numpy as np
        import cv2
        sys.path.insert(0, str(TOOLS))
        try:
            from astra import Astra
            cam = Astra()
        except Exception as e:
            with self.lock:
                self.stats = {'ok': False, 'msg': f'{type(e).__name__}: {str(e)[:60]}'}
            return
        while True:
            try:
                d = cam.depth(wait_ms=800)
                if d is None:
                    time.sleep(0.05); continue
                valid = d > 0
                # 0.3~1.2m 를 색으로 편다 — 작업 영역이 이 대역에 들어온다
                v = np.clip((d.astype(np.float32) - 300) / 900, 0, 1)
                img = cv2.applyColorMap((255 * (1 - v)).astype(np.uint8), cv2.COLORMAP_TURBO)
                img[~valid] = (40, 40, 40)         # 측정 실패는 어둡게
                ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    nz = d[valid]
                    with self.lock:
                        self.jpeg = buf.tobytes()
                        self.stats = {'ok': True,
                                      'valid_pct': round(100 * valid.mean(), 1),
                                      'center_mm': int(d[d.shape[0] // 2, d.shape[1] // 2]),
                                      'min_mm': int(nz.min()) if nz.size else 0,
                                      'msg': ''}
            except Exception as e:
                with self.lock:
                    self.stats = {'ok': False, 'msg': f'{type(e).__name__}'}
                time.sleep(0.3)
            time.sleep(0.08)


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
            elif self.path == '/depth':
                if dep is None:
                    return self._json({'error': 'depth off'}, 503)
                dep.ensure()
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        with dep.lock:
                            j = dep.jpeg
                        if j:
                            self.wfile.write(b'--frame\r\n'
                                             b'Content-Type: image/jpeg\r\n\r\n')
                            self.wfile.write(j); self.wfile.write(b'\r\n')
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif self.path == '/cam':
                if cam is None:
                    return self._json({'error': 'camera off'}, 503)
                cam.ensure()
                # MJPEG 스트림 — 브라우저 <img>가 그대로 재생한다
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        with cam.lock:
                            j = cam.jpeg
                        if j:
                            self.wfile.write(b'--frame\r\n'
                                             b'Content-Type: image/jpeg\r\n\r\n')
                            self.wfile.write(j)
                            self.wfile.write(b'\r\n')
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    pass                          # 탭을 닫으면 여기로 — 정상 종료
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

    dep = Depth() if a.depth else None
    srv = ThreadingHTTPServer(('127.0.0.1', a.http),
                              make_handler(worker, kin, cam, dep))
    print(f'SO-101 패널 → http://127.0.0.1:{a.http}  (시리얼 {port})')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        worker.cmd.put(('disconnect',))
        worker.stop()


if __name__ == '__main__':
    main()
