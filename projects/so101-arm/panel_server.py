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

    # 빨강 HSV 두 구간(색상환 양 끝) — **뎁스캠 전용 값이다.**
    # 손목캠(pick_red.py)의 ((0,150,100),(6,…)) / ((174,150,100),(179,…)) 를 그대로
    # 쓰면 이 카메라에서는 검출 0 이 된다. 2026-08-18 같은 장면 실측:
    #   빨간 물체  H 168~175 (중앙 174) · S 중앙 211 · V 중앙 87
    #   사람 팔    H   5~  6 (중앙   6) · S 중앙 117 · V 중앙 74
    # ① V 하한 100 이 물체(중앙 87)를 통째로 잘라내고 있었다 → 55 로 내린다.
    # ② H 상단 구간이 174~179 라 물체 화소의 절반(168~173)을 놓쳤다 → 166 부터.
    # ③ 살색이 빨강 저채도 쪽에 걸려 사람 팔이 물체보다 큰 덩어리로 잡힌다.
    #    S 하한 140 이면 팔은 전부 빠지고 물체만 남는다(실측: S≥140 에서 팔 0px).
    # 카메라·조명이 바뀌면 다시 잴 것.
    RED = [((0, 140, 55), (10, 255, 255)), ((166, 140, 55), (179, 255, 255))]
    # 뎁스캠에서 큐브는 200px 안팎으로 작게 잡힌다(멀리서 넓게 보므로).
    MIN_AREA = 80
    # 작업 영역 밖(사람·벽)을 거르는 깊이 창 [m]
    Z_RANGE = (0.35, 1.20)

    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.jpeg = None
        self.rgb_jpeg = None
        self.stats = {'ok': False, 'msg': '시작 전'}
        self.blob = None
        self.swap_rb = None        # 컬러가 RGB 로 오는지 BGR 로 오는지 — 첫 검출로 확정
        self.started = False
        self.cam = None
        self._closing = False
        self._color_sig = None
        self._color_same = 0      # 컬러가 몇 프레임째 그대로인가

    def shutdown(self):
        """종료 시 장치를 반드시 놓는다.

        daemon 스레드라 프로세스가 죽으면 close() 가 안 불리는데, Astra 는 그렇게
        버려두면 **다음 프로세스가 열지 못한다**(실측 2026-08-19: 서버를 재시작할
        때마다 카메라만 안 붙어, 포트 전원을 껐다 켜야 복구됐다). 재시작은 연결
        경로에서 토크를 풀어 팔을 떨어뜨리므로 이 정리가 없으면 카메라 하나 때문에
        팔을 내려놓는 사이클이 된다.
        """
        self._closing = True
        c, self.cam = self.cam, None
        if c is not None:
            try:
                c.close()
            except Exception:
                pass

    def ensure(self):
        if not self.started:
            self.started = True
            self.start()

    def run(self):
        import numpy as np
        import cv2
        sys.path.insert(0, str(TOOLS))
        from astra import Astra

        def open_cam():
            """열릴 때까지 계속 시도한다. **포기하지 않는 것이 요점이다.**

            직전 프로세스가 USB 를 놓는 데 시간이 걸려 재기동 직후엔 반드시 몇 번
            실패한다. 종전에는 10회(약 40초) 만에 포기하고 스레드가 끝났는데, 그러면
            복구 수단이 서버 재시작뿐이고 재시작은 연결 경로에서 토크를 풀어 팔을
            떨어뜨린다 — 카메라 하나 때문에 팔을 내려놓는 사이클이 된다(실측
            2026-08-18~19 다섯 차례). 간격을 점증시키며 무한히 기다리는 편이 낫다.
            """
            delay, n = 1.0, 0
            while True:
                n += 1
                try:
                    c = Astra()
                    with self.lock:
                        self.stats = {'ok': False, 'msg': f'열림 (시도 {n})'}
                    return c
                except Exception as e:
                    with self.lock:
                        self.stats = {'ok': False,
                                      'msg': f'열기 대기 {n}회 — {type(e).__name__}'}
                    time.sleep(delay)
                    delay = min(delay * 1.5, 10.0)

        cam = self.cam = open_cam()
        fails = 0
        while True:
            if self._closing:                 # 종료 요청 — 장치를 놓고 나간다
                try:
                    cam.close()
                except Exception:
                    pass
                return
            try:
                d = cam.depth(wait_ms=800)
                if d is None:
                    # 프레임이 계속 안 오면 장치가 빠졌거나 세션이 죽은 것이다.
                    # 여기서 다시 열지 않으면 스레드는 살아 있는데 화면만 굳는다.
                    fails += 1
                    # 임계를 넉넉히 둔다. 짧게 잡으면 일시적 프레임 누락에도 재연결을
                    # 걸어 버리는데, 재연결은 성공률이 100% 가 아니라 멀쩡한 세션을
                    # 잃는 쪽이 손해가 크다(실측 2026-08-19: 60회(≈5초) 임계로 정상
                    # 스트림이 끊겼고, 닫자마자 다시 열려다 9회 연속 실패했다).
                    if fails >= 300:          # ≈ 25초 무프레임
                        with self.lock:
                            self.stats = {'ok': False, 'msg': '프레임 끊김 — 다시 여는 중'}
                        try:
                            cam.close()
                        except Exception:
                            pass
                        self.cam = None
                        time.sleep(3.0)       # USB 가 풀릴 시간 — 없으면 자기 핸들과 충돌
                        cam = self.cam = open_cam()
                        fails = 0
                    time.sleep(0.05); continue
                fails = 0
                valid = d > 0
                # 0.3~1.2m 를 색으로 편다 — 작업 영역이 이 대역에 들어온다
                v = np.clip((d.astype(np.float32) - 300) / 900, 0, 1)
                img = cv2.applyColorMap((255 * (1 - v)).astype(np.uint8), cv2.COLORMAP_TURBO)
                img[~valid] = (40, 40, 40)         # 측정 실패는 어둡게
                self._scan_red(cam, d, cv2, np)
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

    def _scan_red(self, cam, d, cv2, np):
        """컬러에서 빨간 덩어리를 찾고 같은 픽셀의 깊이로 카메라 3D 좌표를 낸다.

        깊이가 컬러 좌표계로 registration 돼 있어야 같은 (u, v) 가 성립한다
        (astra.Astra.registered). 꺼져 있으면 좌표가 조용히 어긋나므로 함께 싣는다.
        """
        rgb = cam.color()
        if rgb is None or rgb.ndim != 3:
            return
        # 컬러가 갱신되는지 추적한다. 깊이만 살아 있고 컬러가 옛 프레임에 고정되면
        # 블롭 좌표가 현재 깊이 맵과 어긋나 cam_xyz 가 조용히 None 이 된다 —
        # 화면만 보면 "멈춘 것 같다"로 끝나므로 수치로 드러내 둔다.
        sig = int(rgb[::32, ::32, 0].sum())
        if sig == self._color_sig:
            self._color_same += 1
        else:
            self._color_sig, self._color_same = sig, 0
        # 미리보기는 검출 결과와 무관하게 갱신한다 — 빨간 물체가 없을 때도 화면은
        # 나와야 조준과 원인 판단을 할 수 있다. swap_rb 미확정이면 RGB 로 가정한다.
        # swap_rb 는 "원본이 RGB라 BGR 로 뒤집어야 한다"는 뜻이다. cv2.imencode 는
        # BGR 을 기대하므로 뒤집은 쪽을 넘겨야 한다 — 종전에는 조건이 반대로 걸려
        # 미리보기에서 R 과 B 가 바뀌어 나왔다(사람 피부가 파랗게 보였다).
        # 검출 경로는 처음부터 옳았고 표시만 틀렸던 것이라, 색으로 원인을 짚기
        # 어려웠다.
        disp = rgb[:, :, ::-1] if self.swap_rb else rgb
        ok, buf = cv2.imencode('.jpg', np.ascontiguousarray(disp),
                               [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self.lock:
                self.rgb_jpeg = buf.tobytes()
        cand = (True, False) if self.swap_rb is None else (self.swap_rb,)
        best = None
        for swap in cand:
            img = rgb[:, :, ::-1] if swap else rgb
            hsv = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_BGR2HSV)
            m = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in self.RED:
                m |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            n, _, st, ct = cv2.connectedComponentsWithStats(m, 8)
            k = max(range(1, n), key=lambda i: st[i, cv2.CC_STAT_AREA], default=None)
            if k is None:
                continue
            area = int(st[k, cv2.CC_STAT_AREA])
            if area >= self.MIN_AREA and (best is None or area > best[0]):
                best = (area, swap, float(ct[k][0]), float(ct[k][1]))
        if best is None:
            with self.lock:
                self.blob = None
            return
        area, swap, u, v = best
        if self.swap_rb is None:
            self.swap_rb = swap        # 한 번 정해지면 이후엔 그 해석만 쓴다
        # 깊이는 한 점만 읽으면 구멍에 걸리므로 블롭 주변 창의 중앙값을 쓴다.
        #
        # 창을 고정 크기로 두면 안 된다. 구조광 방식이라 프로젝터와 카메라의 시차로
        # **물체 주변에 그림자가 생기고**, 작은 물체는 그 그림자에 통째로 들어간다
        # (실측 2026-08-19: 죠 아래 큐브 자리 11x11 창의 유효 화소가 0 이라 z_mm=0,
        # cam_xyz 가 None 이 됐다 — 검출은 정확한데 깊이만 비어 있었다).
        # 유효 화소가 나올 때까지 넓히고, 어디까지 넓혔는지 함께 싣는다.
        h, w = d.shape
        z_mm, nz, used_r = 0, np.array([], dtype=d.dtype), 0
        for r in (5, 9, 14, 20):
            win = d[max(0, int(v) - r):int(v) + r + 1,
                    max(0, int(u) - r):int(u) + r + 1]
            cand = win[win > 0]
            if cand.size >= 8:
                z_mm, nz, used_r = int(np.median(cand)), cand, r
                break
            if cand.size > nz.size:
                nz, used_r = cand, r
        if z_mm == 0 and nz.size:
            z_mm = int(np.median(nz))
        # cam.point() 는 깊이 배열을 통째로 받으므로 여기서 직접 투영한다 —
        # 창 중앙값을 쓰려고 (H, W) 배열을 매 프레임 새로 만들면 낭비가 크다.
        if z_mm:
            z = z_mm / 1000.0
            fx = (w / 2) / math.tan(cam.hfov / 2)
            fy = (h / 2) / math.tan(cam.vfov / 2)
            pt = ((u - w / 2) * z / fx, (v - h / 2) * z / fy, z)
        else:
            pt = None
        if pt is not None and not (self.Z_RANGE[0] <= pt[2] <= self.Z_RANGE[1]):
            pt = None                       # 작업 영역 밖 — 좌표는 버리고 화소만 남긴다
        with self.lock:
            self.blob = {'u': round(u, 1), 'v': round(v, 1), 'area': area,
                         'z_mm': z_mm, 'valid_px': int(nz.size), 'win_r': used_r,
                         'cam_xyz': [round(c, 4) for c in pt] if pt else None,
                         'registered': bool(getattr(cam, 'registered', False)),
                         'swap_rb': bool(self.swap_rb),
                         'color_stale': self._color_same,
                         'color_error': getattr(cam, 'color_error', None)}


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
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        with dep.lock:
                            j = dep.rgb_jpeg
                        if j:
                            self.wfile.write(b'--frame\r\n'
                                             b'Content-Type: image/jpeg\r\n\r\n')
                            self.wfile.write(j); self.wfile.write(b'\r\n')
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    pass
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
