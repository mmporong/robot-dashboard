#!/usr/bin/env python3
"""handeye.py 종단(E2E) 검증 — 실물 없이 전체 파이프라인을 돌린다.

가짜 패널 서버(별도 포트)에 정답 변환(R_true, t_true)을 심고:
  ① 정상 시나리오 — 13점 순회가 완주하고 Kabsch 가 정답을 복원하는지
  ② 스톨 시나리오 — 팔이 안 움직일 때 지점마다 stop 을 보내고 3회에 중단하는지
     (2026-08-19 사고: 이 자리에서 '건너뜀'만 출력하고 계속 밀어 서보를 태웠다)
"""
import json
import math
import pathlib
import sys
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

sys.path.insert(0, str(pathlib.Path('~/so101_tools').expanduser()))
import arm_lib

K = arm_lib.load_kinematics()
MP = arm_lib.load_mapping()
J = arm_lib.JOINTS
PORT = 8865

# 정답 변환: p_rob = R_true · p_cam + t_true
def rot(ax, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if ax == 'x': return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if ax == 'z': return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
R_TRUE = rot('z', 25) @ rot('x', 115)
T_TRUE = np.array([0.42, -0.08, 0.31])
RNG = np.random.default_rng(7)


class FakeArm:
    def __init__(self, stall=False):
        self.stall = stall
        self.pos = {'shoulder_pan': 0.0, 'shoulder_lift': -5.0, 'elbow_flex': 0.0,
                    'wrist_flex': 88.0, 'wrist_roll': 0.0, 'gripper': 40.0}
        self.stops = 0
        self.iks = 0

    def tcp_pan(self):
        q = arm_lib.servo_to_rad({f'{j}.pos': self.pos[j] for j in J}, MP)
        fk = K.fk_pos(q)
        return np.array([v - o for v, o in zip(fk, arm_lib.PAN0)])

    def blob_cam(self):
        p_rob = self.tcp_pan()
        p_cam = R_TRUE.T @ (p_rob - T_TRUE)          # 역변환 + 1mm 잡음
        return (p_cam + RNG.normal(0, 0.001, 3)).round(4).tolist()


def make_handler(arm):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def _json(self, obj, code=200):
            b = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        def do_GET(self):
            if self.path == '/state':
                self._json({'connected': True, 'calibrated': True, 'torque': True,
                            'pos': dict(arm.pos), 'log': []})
            elif self.path == '/blob':
                self._json({'ok': True, 'blob': {'cam_xyz': arm.blob_cam()}})
            else:
                self._json({'error': 'nf'}, 404)
        def do_POST(self):
            n = int(self.headers.get('Content-Length', 0))
            req = json.loads(self.rfile.read(n) or '{}')
            op = req.get('op')
            if op == 'ik':
                arm.iks += 1
                bf = tuple(float(req[k]) + o for k, o in zip('xyz', arm_lib.PAN0))
                q = K.ik_best(*bf, pitch=math.radians(float(req.get('pitch', -90))))
                if q is None:
                    return self._json({'ok': False, 'msg': 'IK 해 없음'})
                if not arm.stall:                     # 스톨 모드면 팔이 안 움직인다
                    tgt = arm_lib.rad_to_servo(q, MP)
                    for k, v in tgt.items():
                        arm.pos[k.replace('.pos', '')] = v
                return self._json({'ok': True, 'q': [round(v, 4) for v in q]})
            if op == 'stop':
                arm.stops += 1
            return self._json({'ok': True})
    return H


def run_case(stall, tag):
    arm = FakeArm(stall=stall)
    srv = ThreadingHTTPServer(('127.0.0.1', PORT), make_handler(arm))
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    import handeye
    handeye.BASE = f'http://127.0.0.1:{PORT}'
    handeye.OUT = pathlib.Path(tempfile.mkdtemp()) / 'handeye.json'
    if stall:                                        # 30초×3 대기를 줄인다
        orig = handeye.wait_reached
        handeye.wait_reached = lambda q, m, **kw: orig(q, m, timeout=2.0)
    sys.argv = ['handeye.py']
    code = None
    try:
        handeye.main()
    except SystemExit as e:
        code = e.code
    srv.shutdown(); srv.server_close()
    return arm, code, handeye.OUT


print('── ① 정상 시나리오: 13점 순회 → Kabsch 복원 ──')
arm, code, out = run_case(False, 'ok')
assert code is None, f'정상 시나리오가 중단됨: {code}'
r = json.loads(out.read_text())
R_est, t_est = np.array(r['R']), np.array(r['t'])
ang = math.degrees(math.acos(min(1.0, (np.trace(R_est @ R_TRUE.T) - 1) / 2)))
dt = np.linalg.norm(t_est - T_TRUE) * 1000
print(f'대응쌍 {r["n"]} · RMS {1000*r["rms_m"]:.1f}mm · 회전 오차 {ang:.2f}° · '
      f'이동 오차 {dt:.1f}mm · 최소 특이값 {min(r["singular_values"]):.4f}')
assert r['n'] == 13 and r['rms_m'] < 0.005
assert ang < 1.0 and dt < 5.0
assert min(r['singular_values']) >= 0.02
print('→ 정답 변환 복원 OK · 특이값 게이트 기준(0.02) 이상\n')

print('── ② 스톨 시나리오: 팔이 안 움직임 (사고 재현 조건) ──')
arm2, code2, _ = run_case(True, 'stall')
print(f'exit code={code2!r} · ik {arm2.iks}회 · stop {arm2.stops}회')
assert code2 is not None and '3회' in str(code2), code2
assert arm2.iks == 3, f'3점에서 중단해야 하는데 {arm2.iks}점 진행'
assert arm2.stops >= 4, f'stop 이 {arm2.stops}회뿐 — 지점마다 + 중단 시 전송돼야 함'
print('→ 도달 실패마다 stop 전송, 3연속 실패에서 순회 중단 OK')
print('   (사고 당시: 13점 전부 "건너뜀"만 출력하고 정지 명령 없이 계속 밀었다)')

# ── 감사 반영 후 추가 시나리오 ─────────────────────────────────────────────
print('\n── ③ 서버 안전장치 발동(토크 킬) 시나리오 ──')
import importlib, handeye as _he
class KillArm(FakeArm):
    def __init__(self):
        super().__init__(stall=False)
        self.torque = True
def make_kill_handler(arm):
    Base = make_handler(arm)
    class H(Base):
        def do_GET(self):
            if self.path == '/state':
                return self._json({'connected': True, 'calibrated': True,
                                   'torque': arm.torque, 'pos': dict(arm.pos),
                                   'log': ['⛔ 스톨 — wrist_roll ... — 토크를 내렸습니다']})
            return Base.do_GET(self)
        def do_POST(self):
            n = int(self.headers.get('Content-Length', 0))
            req = json.loads(self.rfile.read(n) or '{}')
            if req.get('op') == 'ik':
                arm.iks += 1
                if arm.iks == 2:                  # 2번 지점에서 안전장치 발동 재현
                    arm.torque = False
                    return self._json({'ok': True, 'q': [0, 0, 0, 0, 0]})
                bf = tuple(float(req[k]) + o for k, o in zip('xyz', arm_lib.PAN0))
                q = K.ik_best(*bf, pitch=math.radians(-90))
                tgt = arm_lib.rad_to_servo(q, MP)
                for k, v in tgt.items():
                    arm.pos[k.replace('.pos', '')] = v
                return self._json({'ok': True, 'q': [round(v, 4) for v in q]})
            if req.get('op') == 'stop':
                arm.stops += 1
            return self._json({'ok': True})
    return H

import tempfile, threading, time as _t
arm3 = KillArm()
srv = ThreadingHTTPServer(('127.0.0.1', PORT), make_kill_handler(arm3))
threading.Thread(target=srv.serve_forever, daemon=True).start()
import handeye
handeye.BASE = f'http://127.0.0.1:{PORT}'
handeye.OUT = pathlib.Path(tempfile.mkdtemp()) / 'handeye.json'
sys.argv = ['handeye.py']
t0 = _t.monotonic()
code3 = None
try:
    handeye.main()
except SystemExit as e:
    code3 = str(e)
el = _t.monotonic() - t0
srv.shutdown(); srv.server_close()
assert code3 and '안전장치' in code3 and '전원' in code3, code3
assert arm3.iks == 2 and arm3.stops == 0, (arm3.iks, arm3.stops)
assert el < 20, f'{el:.0f}s — 30초×N 헛대기가 남아 있음'
print(f'③ OK — 2번 지점에서 토크 킬 감지, {el:.0f}s 만에 명확한 안내로 중단 (stop 미전송)')

print('\n── ④ 관측 실패 시나리오: stop 없이 건너뛰기 ──')
class BlobFailArm(FakeArm):
    def blob_cam(self):
        if self.iks == 2:                        # 2번 지점만 관측 실패
            return None
        return super().blob_cam()
def make_bf_handler(arm):
    Base = make_handler(arm)
    class H(Base):
        def do_GET(self):
            if self.path == '/blob':
                c = arm.blob_cam()
                return self._json({'ok': c is not None,
                                   'blob': {'cam_xyz': c}})
            return Base.do_GET(self)
    return H
arm4 = BlobFailArm(stall=False)
srv = ThreadingHTTPServer(('127.0.0.1', PORT), make_bf_handler(arm4))
threading.Thread(target=srv.serve_forever, daemon=True).start()
handeye.OUT = pathlib.Path(tempfile.mkdtemp()) / 'handeye.json'
code4 = None
try:
    handeye.main()
except SystemExit as e:
    code4 = str(e)
srv.shutdown(); srv.server_close()
assert code4 is None, code4
r4 = json.loads(handeye.OUT.read_text())
assert r4['n'] == 12 and arm4.stops == 1, (r4['n'], arm4.stops)   # 정리용 1회만
print(f'④ OK — 관측 실패 1점은 stop 없이 건너뜀 (대응쌍 {r4["n"]}, stop 은 종료 정리 1회뿐)')

print('\n── ⑤ 파지 이탈 시나리오: 카메라 좌표 동결 ──')
class DropArm(FakeArm):
    def blob_cam(self):
        if self.iks >= 5:                        # 5번 지점부터 물체가 책상에 떨어짐
            return [-0.150, -0.350, 0.010]       # 고정 좌표 (책상 위 물체)
        return super().blob_cam()
arm5 = DropArm(stall=False)
srv = ThreadingHTTPServer(('127.0.0.1', PORT), make_bf_handler(arm5))
threading.Thread(target=srv.serve_forever, daemon=True).start()
handeye.OUT = pathlib.Path(tempfile.mkdtemp()) / 'handeye.json'
code5 = None
try:
    handeye.main()
except SystemExit as e:
    code5 = str(e)
srv.shutdown(); srv.server_close()
assert code5 and '이탈' in code5, code5
assert arm5.iks <= 8, f'{arm5.iks}점까지 진행 — 이탈 검출이 늦음'
print(f'⑤ OK — {arm5.iks}번 지점에서 이탈 검출·중단: {code5.splitlines()[0][:60]}')
