#!/usr/bin/env python3
"""Orbbec Astra 캡처 전용 데몬 — panel_server 와 분리된 프로세스로 돈다.

## 왜 별도 프로세스인가

Legacy Astra SDK(18.04 바이너리)는 같은 USB2 버스의 UVC 캠이 열리는 순간처럼
대역폭이 출렁일 때 astra_update() 안에서 **영구 교착**할 수 있다(실측
2026-08-19: /cam 최초 접속 순간 깊이 스레드가 C 코드에서 멈춰, 파이썬 쪽
재연결 루프조차 돌지 않았다 — 이때 브라우저는 캐시된 마지막 JPEG 만 받아
"첫 프레임에서 멈춘" 화면이 된다). 교착한 스레드는 같은 프로세스 안에서는
복구할 방법이 없고, 서버 전체 재시작은 팔 토크를 풀어 팔을 떨어뜨린다.

그래서 캡처를 이 데몬으로 떼어냈다. 교착하면 자체 워치독이 프로세스를 끝내고
panel_server 의 감독 스레드가 다시 띄운다 — 그동안 패널과 팔 연결은 무사하다.

    GET /all    → {seq, beat_age, stats, blob, depth_jpeg, rgb_jpeg}  (JPEG 는 base64)
    GET /health → {seq, beat_age}

`seq` 는 새 깊이 프레임마다 1 씩 는다. `beat_age` 는 캡처 루프가 마지막으로
한 바퀴 돈 뒤 지난 시간[s] — 프레임이 없어도(열기 재시도 중) 루프가 살아
있으면 작게 유지되므로, 이 값이 큰 것은 "SDK 안에서 멈춤"만을 뜻한다.
"""
import argparse
import base64
import json
import math
import os
import pathlib
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

TOOLS = pathlib.Path('~/so101_tools').expanduser()
sys.path.insert(0, str(TOOLS))

# 캡처 루프가 이 시간 넘게 한 바퀴도 못 돌면 SDK 교착으로 보고 프로세스를 끝낸다.
# 정상 경로의 최장 블로킹은 Astra() 생성(타임아웃 2s)·depth(wait_ms=800) 이라
# 여유를 크게 둬도 12s 면 충분히 구분된다.
STALE_S = 12.0
# 종료 요청 뒤 이 시간 안에 정리가 안 끝나면 강제 종료 — SIGKILL 을 기다리게
# 하지 않는다 (부모의 grace 타임아웃과 이중 안전망).
CLOSE_GRACE_S = 8.0


class Capture:
    """깊이+컬러를 읽어 최신 JPEG·블롭만 유지한다. (구 panel_server.Depth 본체)"""

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
        self.lock = threading.Lock()
        self.jpeg = None
        self.rgb_jpeg = None
        self.stats = {'ok': False, 'msg': '시작 전'}
        self.blob = None
        self.seq = 0
        self.swap_rb = None        # 컬러가 RGB 로 오는지 BGR 로 오는지 — 첫 검출로 확정
        self.closing = False
        self.closing_t = 0.0
        self.beat = time.monotonic()
        self.cam = None
        self._color_sig = None
        self._color_same = 0      # 컬러가 몇 프레임째 그대로인가
        self._fps = 0.0
        self._fps_n, self._fps_t0 = 0, 0.0

    def request_close(self):
        if not self.closing:
            self.closing_t = time.monotonic()   # 반드시 closing 보다 먼저 — 워치독이
            self.closing = True                 # 그 사이에 보면 t=0 으로 즉시 _exit 한다

    def _open_cam(self, Astra):
        """열릴 때까지 계속 시도한다. **포기하지 않는 것이 요점이다.**

        직전 프로세스가 USB 를 놓는 데 시간이 걸려 재기동 직후엔 반드시 몇 번
        실패한다. 재시도 사이에도 하트비트를 갱신한다 — 열기 대기는 정상 상태라
        워치독이 여기서 프로세스를 끝내면 안 된다.
        """
        delay, n = 1.0, 0
        while not self.closing:
            self.beat = time.monotonic()
            n += 1
            try:
                c = Astra()
                with self.lock:
                    self.stats = {'ok': False, 'msg': f'열림 (시도 {n})'}
                return c
            except Exception as e:
                if n <= 3:                     # 원인은 로그에 남기되 재시도마다 찍진 않는다
                    import traceback
                    traceback.print_exc()
                    sys.stderr.flush()
                with self.lock:
                    self.stats = {'ok': False,
                                  'msg': f'열기 대기 {n}회 — {type(e).__name__}'}
                # 긴 sleep 하나로 두면 그동안 하트비트가 굳는다 — 잘게 쪼갠다
                t_end = time.monotonic() + delay
                while time.monotonic() < t_end and not self.closing:
                    self.beat = time.monotonic()
                    time.sleep(0.2)
                delay = min(delay * 1.5, 10.0)
        return None

    def run(self):
        import numpy as np
        import cv2
        from astra import Astra

        cam = self.cam = self._open_cam(Astra)
        if cam is None:                       # closing 중 — 열지도 못하고 끝
            return
        fails = 0
        self._fps_t0 = time.monotonic()
        while True:
            self.beat = time.monotonic()
            if self.closing:                  # 종료 요청 — 장치를 놓고 나간다
                try:
                    cam.close()
                except Exception:
                    pass
                self.cam = None
                return
            try:
                d = cam.depth(wait_ms=800)
                if d is None:
                    # 프레임이 계속 안 오면 장치가 빠졌거나 세션이 죽은 것이다.
                    fails += 1
                    # 임계를 넉넉히 둔다. 짧게 잡으면 일시적 프레임 누락에도 재연결을
                    # 걸어 버리는데, 재연결은 성공률이 100% 가 아니라 멀쩡한 세션을
                    # 잃는 쪽이 손해가 크다(실측 2026-08-19: 60회(≈5초) 임계로 정상
                    # 스트림이 끊겼고, 닫자마자 다시 열려다 9회 연속 실패했다).
                    if fails >= 30:           # ≈ 25초 무프레임 (depth 가 800ms 씩 기다림)
                        with self.lock:
                            self.stats = {'ok': False, 'msg': '프레임 끊김 — 다시 여는 중'}
                        try:
                            cam.close()
                        except Exception:
                            pass
                        self.cam = None
                        time.sleep(3.0)       # USB 가 풀릴 시간 — 없으면 자기 핸들과 충돌
                        cam = self.cam = self._open_cam(Astra)
                        if cam is None:
                            return
                        fails = 0
                    if self.closing:
                        continue              # 루프 머리에서 정리한다
                    time.sleep(0.05)
                    continue
                fails = 0
                # 실제 카메라 프레임 레이트를 센다. 낮으면 화면이 정지처럼 보여
                # "깊이가 멈췄다"로 오해하게 되므로 수치로 드러내 둔다.
                self._fps_n += 1
                now = time.monotonic()
                if now - self._fps_t0 >= 2.0:
                    self._fps = round(self._fps_n / (now - self._fps_t0), 1)
                    self._fps_n, self._fps_t0 = 0, now
                valid = d > 0
                # 0.3~1.2m 를 색으로 편다 — 작업 영역이 이 대역에 들어온다
                v = np.clip((d.astype(np.float32) - 300) / 900, 0, 1)
                img = cv2.applyColorMap((255 * (1 - v)).astype(np.uint8), cv2.COLORMAP_TURBO)
                img[~valid] = (40, 40, 40)         # 측정 실패는 어둡게
                # 살아 있음을 화면에 적는다. 깊이 센서는 mm 로 양자화돼 있어 장면이
                # 정적이면 프레임이 바이트 단위로 동일해진다 — 타임스탬프가 없으면
                # 정지 화면과 구분되지 않는다.
                stamp = time.strftime('%H:%M:%S')
                cv2.putText(img, f'{stamp}  {self._fps:.0f}fps', (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(img, f'{stamp}  {self._fps:.0f}fps', (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
                self._scan_red(cam, d, cv2, np)
                ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    nz = d[valid]
                    with self.lock:
                        self.jpeg = buf.tobytes()
                        self.seq += 1
                        self.stats = {'ok': True,
                                      'valid_pct': round(100 * valid.mean(), 1),
                                      'center_mm': int(d[d.shape[0] // 2, d.shape[1] // 2]),
                                      'min_mm': int(nz.min()) if nz.size else 0,
                                      'fps': self._fps,
                                      'msg': ''}
            except Exception as e:
                import traceback
                traceback.print_exc()          # stderr → depth_daemon.log
                sys.stderr.flush()
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
        # 블롭 좌표가 현재 깊이 맵과 어긋나 cam_xyz 가 조용히 None 이 된다.
        sig = int(rgb[::32, ::32, 0].sum())
        if sig == self._color_sig:
            self._color_same += 1
        else:
            self._color_sig, self._color_same = sig, 0
        # 미리보기는 검출 결과와 무관하게 갱신한다. swap_rb 는 "원본이 RGB 라 BGR 로
        # 뒤집어야 한다"는 뜻 — cv2.imencode 는 BGR 을 기대한다.
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
        # 구조광 그림자에 작은 물체가 통째로 들어갈 수 있어(실측 2026-08-19:
        # 11x11 창 유효 화소 0), 유효 화소가 나올 때까지 창을 넓힌다.
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


def make_handler(cap):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            age = round(time.monotonic() - cap.beat, 1)
            if self.path == '/all':
                # 락 안에서는 참조만 복사한다 — dumps·소켓 전송까지 락을 쥐면 느린
                # 클라이언트가 캡처 루프를 세우고, 하트비트가 굳어 워치독이 멀쩡한
                # 데몬을 죽인다. (캡처는 stats·blob 을 통째로 새 dict 로 갈아끼우고
                # jpeg 는 bytes 라, 참조만 들고 나와도 안전하다.)
                with cap.lock:
                    seq, stats, blob = cap.seq, cap.stats, cap.blob
                    jpeg, rgb = cap.jpeg, cap.rgb_jpeg
                self._json({'seq': seq, 'beat_age': age, 'stats': stats,
                            'blob': blob,
                            'depth_jpeg': base64.b64encode(jpeg).decode()
                                          if jpeg else None,
                            'rgb_jpeg': base64.b64encode(rgb).decode()
                                        if rgb else None})
            elif self.path == '/health':
                self._json({'seq': cap.seq, 'beat_age': age})
            else:
                self._json({'error': 'not found'}, 404)

    return H


def _watchdog(cap):
    """캡처 루프가 SDK 안에서 굳으면 프로세스를 끝낸다 — 부모가 다시 띄운다.

    굳은 스레드에서 cam.close() 를 부르는 것은 위험하다(다른 스레드가 SDK 안에
    있는 채로 세션을 무너뜨리면 double free — 실측 2026-08-18 장치가 전원을
    끊어야 풀리는 상태가 됐다). 그래서 정리 시도 없이 즉시 _exit 한다. 커널이
    usbfs 핸들을 회수하고, 다음 프로세스의 열기 재시도 루프가 이어받는다.
    """
    while True:
        time.sleep(2.0)
        now = time.monotonic()
        if os.getppid() == 1:
            # 부모(panel_server)가 SIGKILL·크래시로 사라짐. 고아로 남으면 Astra 를
            # 영구 점유해 다음 기동의 bind·open 이 전부 막힌다 — 스스로 나간다.
            os._exit(4)
        if cap.closing:
            if now - cap.closing_t > CLOSE_GRACE_S:
                os._exit(2)
            continue
        if now - cap.beat > STALE_S:
            sys.stderr.write(f'[watchdog] 캡처 루프 {now - cap.beat:.1f}s 정지 — 재기동 위해 종료\n')
            sys.stderr.flush()
            os._exit(3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--http', type=int, default=8766)
    a = ap.parse_args()

    cap = Capture()
    srv = ThreadingHTTPServer(('127.0.0.1', a.http), make_handler(cap))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=_watchdog, args=(cap,), daemon=True).start()

    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: cap.request_close())

    print(f'depth_daemon → http://127.0.0.1:{a.http}', flush=True)
    try:
        cap.run()      # 메인 스레드가 캡처 — closing 시 스스로 닫고 돌아온다
    except Exception:
        # import 실패·환경 어긋남 같은 치명 오류. 즉시 죽으면 부모의 재기동
        # 루프에 합류해 원인이 로그 더미에 묻힌다 — 30초 살아서 /all 로 이유를
        # 노출한 뒤 종료해 부모의 백오프 재기동을 태운다.
        import traceback
        traceback.print_exc()
        sys.stderr.flush()
        with cap.lock:
            cap.stats = {'ok': False, 'msg': '캡처 치명 오류 — depth_daemon.log 확인'}
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30.0 and not cap.closing:
            cap.beat = time.monotonic()      # 워치독 오탐 방지
            time.sleep(0.5)
        sys.exit(5)


if __name__ == '__main__':
    main()
