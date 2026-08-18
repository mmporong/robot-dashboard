#!/usr/bin/env python3
"""뎁스캠 ↔ 로봇 좌표 정합 — 죠에 물린 빨간 물체로 대응쌍을 모아 강체 변환을 푼다.

## 무엇을 구하나

뎁스캠이 본 점 `p_cam` 을 로봇 좌표 `p_rob` 로 옮기는 회전·평행이동 (R, t):

    p_rob = R · p_cam + t

카메라를 고정해 두고 팔을 여러 지점으로 보내며, 각 지점에서
  · 로봇 좌표 = 명령한 IK 목표(= FK 로 검증된 TCP 위치)
  · 카메라 좌표 = 죠에 물린 빨간 물체의 (u, v) 와 그 자리의 깊이
를 짝지어 기록한 뒤 Kabsch(SVD) 로 (R, t) 를 닫힌 형태로 푼다.

## 죠-물체 오프셋을 따로 재지 않는 이유

물체는 죠 사이 한 자리에 계속 물려 있으므로 TCP 와의 상대 위치가 **상수**다.
그 상수는 t 에 그대로 흡수되고, 우리가 얻는 변환은 "카메라가 본 물체 →
그 물체를 물고 있던 TCP 목표"가 된다. 나중에 책상 위 물체를 잡을 때 필요한
것도 정확히 그 값이라, 오프셋을 따로 재는 단계가 사라진다.

## 지점 선정

한 평면에 몰리면 SVD 가 퇴화해 회전이 안 정해진다. x·y·z 를 모두 흔들고
(특히 z 를 두 층 이상) 최소 6점, 권장 10점 이상을 모은다.

사용:
    python3 handeye.py                 # 기본 격자로 수집 → 계산 → 저장
    python3 handeye.py --dry           # 이동 없이 현재 프레임만 확인
"""
import argparse
import json
import math
import pathlib
import sys
import time
import urllib.request

import numpy as np

BASE = 'http://127.0.0.1:8765'
HERE = pathlib.Path(__file__).parent
OUT = HERE / 'handeye.json'

# 작업 영역 안에서 세 축을 모두 흔든 격자. z 를 두 층으로 둬 평면 퇴화를 막는다.
POSES = [
    (0.18, -0.06, 0.02), (0.18, 0.00, 0.02), (0.18, 0.06, 0.02),
    (0.23, -0.06, 0.02), (0.23, 0.00, 0.02), (0.23, 0.06, 0.02),
    (0.18, -0.04, 0.09), (0.18, 0.04, 0.09),
    (0.23, -0.04, 0.09), (0.23, 0.04, 0.09),
    (0.205, 0.00, 0.055),
]


def post(op, **kw):
    req = urllib.request.Request(
        f'{BASE}/cmd', data=json.dumps(dict(op=op, **kw)).encode(),
        headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(req, timeout=15).read())


def get(path):
    return json.loads(urllib.request.urlopen(f'{BASE}{path}', timeout=15).read())


def read_blob(tries=12, need=5):
    """블롭이 안정될 때까지 여러 프레임을 읽어 중앙값을 쓴다.

    한 프레임만 믿으면 깊이 잡음과 순간적인 오검출이 그대로 대응쌍에 들어간다.
    """
    pts = []
    for _ in range(tries):
        r = get('/blob')
        b = r.get('blob')
        if b and b.get('cam_xyz'):
            pts.append(b['cam_xyz'])
        time.sleep(0.15)
    if len(pts) < need:
        return None, len(pts)
    a = np.array(pts)
    med = np.median(a, axis=0)
    # 중앙값에서 2cm 넘게 떨어진 관측은 버리고 다시 평균 낸다
    keep = a[np.linalg.norm(a - med, axis=1) < 0.02]
    if len(keep) < need:
        return None, len(keep)
    return keep.mean(axis=0), len(keep)


def kabsch(P, Q):
    """P(카메라) → Q(로봇) 강체 변환. 반환 (R, t, rms)."""
    pc, qc = P.mean(0), Q.mean(0)
    H = (P - pc).T @ (Q - qc)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = qc - R @ pc
    err = (P @ R.T + t) - Q
    return R, t, float(np.sqrt((err ** 2).sum(1).mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry', action='store_true', help='이동 없이 현재 관측만 확인')
    ap.add_argument('--settle', type=float, default=1.2, help='이동 후 안정 대기 [s]')
    a = ap.parse_args()

    st = get('/state')
    if not (st['connected'] and st['calibrated']):
        sys.exit('연결·캘리브레이션이 먼저 필요합니다')

    if a.dry:
        p, n = read_blob()
        print(f'관측 {n} 프레임 · 카메라 좌표 {None if p is None else np.round(p, 4)}')
        return

    if not st['torque']:
        sys.exit('토크 ON 후에 실행하세요')

    cam_pts, rob_pts, log = [], [], []
    for i, (x, y, z) in enumerate(POSES, 1):
        r = post('ik', x=x, y=y, z=z, pitch=-90)
        if not r.get('ok'):
            print(f'[{i:2d}/{len(POSES)}] ({x}, {y}, {z}) IK 실패 — {r.get("msg")}')
            continue
        time.sleep(3.2 + a.settle)          # 보간 이동 3초 + 진동 안정
        p, n = read_blob()
        if p is None:
            print(f'[{i:2d}/{len(POSES)}] ({x:+.3f},{y:+.3f},{z:+.3f}) 관측 실패 (유효 {n})')
            continue
        cam_pts.append(p)
        rob_pts.append(r['fk_pan'])
        log.append({'target': [x, y, z], 'fk': r['fk_pan'],
                    'cam': [round(v, 4) for v in p], 'frames': n})
        print(f'[{i:2d}/{len(POSES)}] 로봇 ({x:+.3f},{y:+.3f},{z:+.3f}) '
              f'↔ 카메라 ({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})  [{n}프레임]')

    if len(cam_pts) < 6:
        sys.exit(f'대응쌍이 {len(cam_pts)}개뿐입니다 — 최소 6개가 필요합니다')

    P, Q = np.array(cam_pts), np.array(rob_pts)
    R, t, rms = kabsch(P, Q)
    # 점들이 한 평면에 몰렸는지 — 가장 작은 특이값이 회전을 정하는 힘이다
    sv = np.linalg.svd(P - P.mean(0), compute_uv=False)
    print(f'\n대응쌍 {len(P)}개 · RMS 잔차 {1000*rms:.1f} mm')
    print(f'점 분포 특이값 {np.round(sv, 4)}  (막내가 0.02 미만이면 평면 퇴화 위험)')
    print('R =\n', np.round(R, 4))
    print('t =', np.round(t, 4))

    OUT.write_text(json.dumps({
        'R': R.tolist(), 't': t.tolist(), 'rms_m': rms, 'n': len(P),
        'singular_values': sv.tolist(), 'samples': log,
        'note': ('2026-08-18 뎁스캠↔로봇 정합. 죠에 빨간 물체를 물린 채 여러 지점으로 '
                 '이동하며 (IK 목표, 카메라 관측) 쌍을 모아 Kabsch 로 풀었다. '
                 'p_rob = R·p_cam + t. 물체가 죠에서 차지하는 상대 위치는 상수라 t 에 '
                 '흡수돼 있어, 이 변환은 곧 "카메라가 본 물체 → 그것을 물 TCP 목표"다. '
                 '카메라나 베이스를 움직이면 다시 잴 것.'),
    }, ensure_ascii=False, indent=2))
    print(f'\n저장: {OUT}')


if __name__ == '__main__':
    main()
