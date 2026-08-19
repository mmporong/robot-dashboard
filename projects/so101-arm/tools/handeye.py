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
# z=+0.02 는 x≤0.22 에서만 풀린다.
#
# ✎ 아래층은 -0.03 — 책상면 실측 -0.078 (2026-08-19 저녁 확정) 기준 여유 48mm 로,
# 죠에 물린 물체가 ~20mm 아래로 튀어나와도 28mm 가 남는다. 종전 -0.05 는 구
# floor(-0.1037) 기준 설계라 새 floor 에서는 실여유가 8mm 로 접촉 사거리였다
# (리뷰 M6-2). z 층 분산이 5cm 로 줄어드는 대신, 퇴화는 실행 시 최소 특이값
# 출력으로 확인한다. preflight 가 바닥 여유(MIN_CLEAR)를 정적으로 검사한다.
POSES = [
    (0.18, -0.06, -0.03), (0.18, 0.00, -0.03), (0.18, 0.06, -0.03),
    (0.23, -0.06, -0.03), (0.23, 0.00, -0.03), (0.23, 0.06, -0.03),
    (0.18, -0.05, -0.01), (0.18, 0.05, -0.01),
    (0.23, -0.05, -0.01), (0.23, 0.05, -0.01),
    (0.19, -0.03,  0.02), (0.19, 0.03,  0.02),
    (0.21, 0.00,  0.02),
]

# 아래층과 책상면 사이에 요구하는 최소 여유 [m] — 물체 돌출(~0.02) + 바닥 불확실
# (±0.002) + 자세 오차 여유. preflight 가 floor_z_m 실측값과 대조한다.
# ✎ 현 구성(-0.03, floor -0.078)의 실여유는 48mm 로 **헤드룸이 3mm 뿐**이다 —
# 바닥 재측정값이 -0.075 위로 나오면 여기서 막힌다. 그건 오류가 아니라 설계
# 재검토 신호다(아래층을 -0.025 로 올리는 식으로). 리뷰 m32.
MIN_CLEAR_M = 0.045


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
        st = get('/state')
        pos = st['pos']
        # 서버 안전장치가 토크를 내렸으면 더 기다릴 이유가 없다 — 즉시 반환해
        # 호출부의 상태 검사로 넘긴다(종전엔 지점당 30초씩 헛기다렸다).
        if not (st.get('torque', True) and st.get('connected', True)):
            return pos, gap_of(pos)
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

    ★ 품질 게이트(2026-08-19 1차 순회 실측): 구조광 그림자로 물체 위 유효
    깊이 화소가 없으면 데몬이 창을 r=14~20 까지 넓혀 **주변(책상) 화소의
    중앙값**을 쓴다 — 그 값은 물체가 아니라 배경 깊이라, RMS 33mm 의 계통
    오차가 그대로 들어왔다. 창이 작고(r≤9) 물체 화소가 충분한(≥8) 프레임만
    채택한다. 대부분 탈락하면 물체가 깊이 센서에 너무 작은 것 — 더 큰 물체로.
    """
    pts = []
    rejected = 0
    for _ in range(tries):
        r = get('/blob')
        b = r.get('blob')
        if b and b.get('cam_xyz'):
            if b.get('win_r', 99) <= 9 and b.get('valid_px', 0) >= 8:
                pts.append(b['cam_xyz'])
            else:
                rejected += 1
        time.sleep(0.15)
    if rejected and len(pts) < need:
        print(f'    (깊이 품질 미달 {rejected}프레임 탈락 — 물체가 깊이 센서에 '
              f'너무 작습니다. win_r≤9·valid_px≥8 요구)')
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


def preflight(kin, mapping):
    """이동을 시작하기 **전에** 13점 전부를 IK 로 풀어 캘리브 범위와 대조한다.

    한 점이라도 범위 밖이면 순회 중간에 이동 거부(또는 스톨)로 끊긴다 — 그때는
    이미 팔이 움직인 뒤다. 캘리브 파일이 바뀔 때마다 도달성이 달라지므로
    (2026-08-19 재캘리브), 실물을 움직이기 전에 전 지점을 정적으로 확인한다.
    """
    calib_p = pathlib.Path('~/.cache/huggingface/lerobot/calibration/robots/'
                           'so_follower/follower.json').expanduser()
    try:
        cal = json.loads(calib_p.read_text())
    except Exception as e:
        # fail-closed — "이동 전 정적 검증"이 목적인데 파일이 없다고 그냥
        # 진행하면 이 함수가 IK 해 존재 확인으로 조용히 축소된다.
        sys.exit(f'캘리브 파일을 읽지 못해 범위 검증을 할 수 없습니다'
                 f'({type(e).__name__}): {calib_p}\n'
                 f'서버를 다른 --id 로 띄웠다면 이 경로부터 맞추세요.')
    # 경계 식은 arm_lib.calib_bounds — lerobot DEGREES 정규화와 같은 식 하나만 쓴다.
    bounds = arm_lib.calib_bounds(cal)
    margin = 2.0                       # arm_gui.LIMIT_MARGIN_DEG 와 같은 값 (수동 동기)
    bad = []
    for x, y, z in POSES:
        bf = tuple(p + o for p, o in zip((x, y, z), arm_lib.PAN0))
        q = kin.ik_best(*bf, pitch=math.radians(-90))
        if q is None:
            bad.append(f'({x:+.2f},{y:+.2f},{z:+.2f}) IK 해 없음')
            continue
        tgt = arm_lib.rad_to_servo(q, mapping)
        for j in JOINTS:
            v = tgt[f'{j}.pos']
            lo = bounds[j][0] + margin
            hi = bounds[j][1] - margin
            if not (lo <= v <= hi):
                bad.append(f'({x:+.2f},{y:+.2f},{z:+.2f}) {j}={v:+.1f}° '
                           f'가 캘리브 범위({lo:+.1f}~{hi:+.1f}) 밖')
    if bad:
        sys.exit('프리플라이트 실패 — 아래 지점이 도달 불가입니다. POSES 를 고치세요:\n'
                 + '\n'.join('  · ' + b for b in bad))
    # 바닥 여유 — 상수의 유효성(stale)만이 아니라 **유도값**(여유)도 검사한다.
    # floor 가 바뀌면 아래층 설계 근거가 함께 바뀌는데, stale 게이트는 그걸 못
    # 본다(리뷰 M6-2: floor -0.1037→-0.078 로 실여유가 8mm 까지 줄었었다).
    try:
        floor = arm_lib.load_gain('floor_z_m')['floor_z_m']
    except (SystemExit, OSError, ValueError, KeyError):
        floor = None                # stale/부재 — main 의 --force-floor 게이트 담당
                                    # (BaseException 은 Ctrl-C 까지 삼킨다 — 리뷰 m31)
    if floor is not None:
        zmin = min(p[2] for p in POSES)
        clear = zmin - floor
        if clear < MIN_CLEAR_M:
            sys.exit(f'프리플라이트 실패 — 아래층 z={zmin} 와 책상면 {floor} 의 '
                     f'여유 {clear*1000:.0f}mm < {MIN_CLEAR_M*1000:.0f}mm. '
                     f'POSES 아래층을 올리세요')
        print(f'바닥 여유 {clear*1000:.0f}mm (책상 {floor}, 요구 {MIN_CLEAR_M*1000:.0f}mm)')
    print(f'프리플라이트 통과 — {len(POSES)}점 전부 IK 해 있음 · 캘리브 범위 안')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry', action='store_true', help='이동 없이 현재 관측만 확인')
    ap.add_argument('--settle', type=float, default=1.2, help='이동 후 안정 대기 [s]')
    ap.add_argument('--force-floor', action='store_true',
                    help='floor_z_m 이 무효(stale)여도 아래층 z=-0.05 를 강행한다')
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

    # 아래층 z=-0.05 는 "책상면에서 5cm 여유"가 근거인데, 그 근거(floor_z_m)가
    # 재캘리브로 무효면 실제 여유를 모른다 — 물린 물체가 책상에 닿으면 사고 재연이다.
    if not a.force_floor:
        try:
            arm_lib.load_gain('floor_z_m')
        except SystemExit as e:
            sys.exit(f'{e}\n→ probe_floor.py 로 책상 높이를 다시 잰 뒤 실행하거나, '
                     f'여유를 눈으로 확인했으면 --force-floor 로 강행하세요.')
        except Exception as e:
            # servo_gain.json 이 없거나 깨져도 가드는 fail-closed 다
            sys.exit(f'servo_gain.json 을 읽지 못해 floor_z_m 유효성을 확인할 수 '
                     f'없습니다({type(e).__name__}: {e}) — --force-floor 로만 강행 가능')

    kin = arm_lib.load_kinematics()
    mapping = arm_lib.load_mapping()
    preflight(kin, mapping)             # 실물을 움직이기 전에 전 지점 정적 검증
    cam_pts, rob_pts, log = [], [], []
    stalls = 0                    # 연속 도달 실패 횟수 — 쌓이면 중단한다
    prev_pair = None              # 파지 이탈 검출용 (직전 로봇·카메라 좌표)
    loose = 0
    for i, (x, y, z) in enumerate(POSES, 1):
        r = post('ik', x=x, y=y, z=z, pitch=-90)
        if not r.get('ok'):
            print(f'[{i:2d}/{len(POSES)}] ({x}, {y}, {z}) IK 실패 — {r.get("msg")}')
            continue
        pos, gap = wait_reached(r['q'], mapping)   # 목표 관절각에 도달할 때까지
        # 서버 쪽 사정부터 본다 — 안전장치가 이미 토크를 내렸으면(스톨 킬 등)
        # stop 도 재시도도 의미가 없고, 침묵 속에 헛도는 것이 사고 당시의
        # "아무도 몰랐다"다(감사 ③④). gap 분기 **밖**에서 검사한다 — 보간
        # 막바지에 킬이 나면 gap ≤ 3 으로 빠져나와 검사를 건너뛸 수 있다(n1).
        st2 = get('/state')
        if not (st2.get('connected') and st2.get('torque')):
            tail = ' / '.join(st2.get('log', [])[-2:])
            sys.exit(f'[{i:2d}/{len(POSES)}] 서버 안전장치 발동 또는 통신 이상 '
                     f'(connected={st2.get("connected")} torque={st2.get("torque")})\n'
                     f'  최근 로그: {tail}\n'
                     f'  → 토크가 내려간 것이면 원인 해소 후 재시작. 응답이 아예 '
                     f'없으면 서보 전원을 차단하세요 (눌림 지속 시 버스가 죽는다).')
        if gap > 3.0:
            # ★ 반드시 정지시킨다. 목표를 그대로 두면 서보가 막힌 방향으로 계속
            # 밀어 탄다 — 2026-08-19 이 자리에서 "건너뜀"만 출력해 서보를 태웠다.
            # 실패를 기록하는 것과 힘을 빼는 것은 다른 일이다.
            post('stop')
            print(f'[{i:2d}/{len(POSES)}] 관절이 목표에서 {gap:.1f}° 남음 — 정지하고 건너뜀')
            stalls += 1
            if stalls >= 3:
                post('stop')
                sys.exit('연속 도달 실패가 3회를 넘었습니다 — 간섭을 확인하세요. '
                         '계속 밀면 서보가 탑니다.')
            continue
        stalls = 0
        time.sleep(a.settle)                # 진동 가라앉힘
        rob = fk_of(pos, kin, mapping)      # **실제** 도달 위치
        p, n = read_blob()
        err = max(abs(c - t) for c, t in zip(rob, (x, y, z)))
        if p is None:
            # ★ 관측 실패는 이동 실패가 아니다 — stop 을 보내지 않는다(감사 M1).
            # stop 은 ①위치 재전송으로 펌웨어 보호 플래그를 풀고 ②(수정 전에는)
            # 그리퍼 예압을 지웠으며 ③속도를 8 로 내려 다음 이동을 오탐 킬로
            # 몰았다(C1). 팔은 도달 자세를 목표=도달점으로 유지 중이라 안전하다.
            print(f'[{i:2d}/{len(POSES)}] ({x:+.3f},{y:+.3f},{z:+.3f}) 관측 실패 (유효 {n})')
            continue
        if err > 0.02:
            print(f'[{i:2d}/{len(POSES)}] 목표와 {1000*err:.0f}mm 어긋나 건너뜀 '
                  f'(도달 {rob})')
            continue
        # ★ 파지 이탈 검출 — 물체가 죠에서 빠지면 책상 위 빨간 블롭은 계속
        # 보이므로 read_blob 은 성공한다. 그러면 13점이 조용히 오염된다(감사 M8).
        # 로봇이 수 cm 움직였는데 카메라 좌표가 안 따라오면 물체가 팔에 없는 것.
        if prev_pair is not None:
            drob = float(np.linalg.norm(np.array(rob) - np.array(prev_pair[0])))
            dcam = float(np.linalg.norm(np.array(p) - np.array(prev_pair[1])))
            if drob > 0.02 and dcam < 0.3 * drob:
                loose += 1
                if loose >= 2:
                    sys.exit(f'[{i:2d}/{len(POSES)}] 물체 이탈 의심 — 로봇 이동 '
                             f'{1000*drob:.0f}mm 에 카메라 이동 {1000*dcam:.0f}mm '
                             f'(2회 연속). 물체를 다시 물리고 재시작하세요.')
            else:
                loose = 0
        prev_pair = (rob, list(p))
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
    post('stop')                  # 끝났으면 남은 목표를 지운다


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        # Ctrl-C 로 빠져도 팔을 정리한다 — 정지 없이 스크립트만 사라지면
        # 진행 중이던 목표가 남는다(감사 M6). stop 은 ARM 목표만 현재 위치로
        # 덮으므로(그리퍼 예압 유지) 안전하다.
        try:
            post('stop')
        except Exception:
            pass
        sys.exit('\n사용자 중단 — 정지를 보냈습니다 (토크 유지, 현재 자세 고정)')
    except Exception:
        # 예기치 못한 예외도 정지 시도 후 원래 트레이스백을 살려 던진다(n2).
        # SystemExit 은 BaseException 이라 여기 안 걸린다 — 정상 종료 경로 유지.
        try:
            post('stop')
        except Exception:
            pass
        raise
