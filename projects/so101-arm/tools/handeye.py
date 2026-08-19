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
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
import arm_lib                                    # noqa: E402
OUT = HERE / 'handeye.json'

# 작업 영역 안에서 세 축을 모두 흔든 격자. z 를 **세 층**으로 둬 평면 퇴화를 막는다.
#
# 범위는 실제 IK 도달성을 계산해 정했다(2026-08-19): z=+0.04 이상은 리치 밖이고,
# z=+0.02 는 x≤0.22 에서만 풀린다. 아래층은 책상면(floor_z=-0.1037)에서 5cm 여유를
#둔 -0.05 로 잡았다 — 죠에 물린 물체가 아래로 튀어나와 있어 더 내리면 닿는다.
POSES = [
    (0.18, -0.06, -0.05), (0.18, 0.00, -0.05), (0.18, 0.06, -0.05),
    (0.23, -0.06, -0.05), (0.23, 0.00, -0.05), (0.23, 0.06, -0.05),
    (0.18, -0.05, -0.01), (0.18, 0.05, -0.01),
    (0.23, -0.05, -0.01), (0.23, 0.05, -0.01),
    (0.19, -0.03,  0.02), (0.19, 0.03,  0.02),
    (0.21, 0.00,  0.02),
]


def post(op, **kw):
    req = urllib.request.Request(
        f'{BASE}/cmd', data=json.dumps(dict(op=op, **kw)).encode(),
        headers={'Content-Type': 'application/json'})
    return json.loads(urllib.request.urlopen(req, timeout=15).read())


def get(path):
    return json.loads(urllib.request.urlopen(f'{BASE}{path}', timeout=15).read())


def wait_reached(q_rad, mapping, timeout=30.0, tol=1.0, hold=3):
    """IK 가 낸 목표 관절각에 실제로 도달할 때까지 기다린다.

    두 가지를 다 틀렸던 자리다.
    ① IK 목표를 그대로 로봇 좌표로 쓰면 안 된다 — 명령이지 도달 위치가 아니다.
       속도 제한이 걸린 채 큰 이동을 주면 보간 시간(3초)이 지나도 팔이 계속 가고
       있어, 고정 대기 후 측정하면 직전 위치를 그 지점의 값으로 기록하게 된다
       (실측 2026-08-19: 13점 중 9점의 카메라 좌표가 3mm 안에 뭉쳐 RMS 52mm).
    ② "관절 변화가 멈추면 도착"으로 판단해서도 안 된다. 명령 직후엔 아직 출발
       전이라 변화가 0 이고, 그대로 "이미 도착"으로 읽어 13점 전부 같은 자리에서
       측정했다. **목표값과의 거리**로 판정해야 한다.
    """
    want = arm_lib.rad_to_servo(q_rad, mapping)
    want = {k.replace('.pos', ''): v for k, v in want.items()}

    def gap_of(pos):
        # ±180 은 같은 자세다. 정규화하지 않으면 wrist_roll 목표 -180 과 실제 +180 이
        # 278° 차이로 읽혀 도달을 영영 인정하지 못한다(실측 2026-08-19).
        return max(abs((pos[j] - want[j] + 180) % 360 - 180) for j in JOINTS)

    near = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        pos = get('/state')['pos']
        gap = gap_of(pos)
        if gap < tol:
            near += 1
            if near >= hold:
                return pos, gap
        else:
            near = 0
        time.sleep(0.3)
    pos = get('/state')['pos']
    return pos, gap_of(pos)


def fk_of(pos, kin, mapping):
    """관측된 관절각 → pan 축 기준 TCP 좌표 [m]."""
    obs = {f'{j}.pos': pos[j] for j in JOINTS}
    q = arm_lib.servo_to_rad(obs, mapping)
    fk = kin.fk_pos(q)
    return [round(v - o, 5) for v, o in zip(fk, arm_lib.PAN0)]


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

    kin = arm_lib.load_kinematics()
    mapping = arm_lib.load_mapping()
    cam_pts, rob_pts, log = [], [], []
    for i, (x, y, z) in enumerate(POSES, 1):
        r = post('ik', x=x, y=y, z=z, pitch=-90)
        if not r.get('ok'):
            print(f'[{i:2d}/{len(POSES)}] ({x}, {y}, {z}) IK 실패 — {r.get("msg")}')
            continue
        pos, gap = wait_reached(r['q'], mapping)   # 목표 관절각에 도달할 때까지
        if gap > 3.0:
            print(f'[{i:2d}/{len(POSES)}] 관절이 목표에서 {gap:.1f}° 남아 건너뜀')
            continue
        time.sleep(a.settle)                # 진동 가라앉힘
        rob = fk_of(pos, kin, mapping)      # **실제** 도달 위치
        p, n = read_blob()
        err = max(abs(c - t) for c, t in zip(rob, (x, y, z)))
        if p is None:
            print(f'[{i:2d}/{len(POSES)}] ({x:+.3f},{y:+.3f},{z:+.3f}) 관측 실패 (유효 {n})')
            continue
        if err > 0.02:
            print(f'[{i:2d}/{len(POSES)}] 목표와 {1000*err:.0f}mm 어긋나 건너뜀 '
                  f'(도달 {rob})')
            continue
        cam_pts.append(p)
        rob_pts.append(rob)
        log.append({'target': [x, y, z], 'reached': rob,
                    'cam': [round(v, 4) for v in p], 'frames': n})
        print(f'[{i:2d}/{len(POSES)}] 로봇 ({rob[0]:+.3f},{rob[1]:+.3f},{rob[2]:+.3f}) '
              f'↔ 카메라 ({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})  [{n}프레임, 오차 {1000*err:.0f}mm]')

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
