#!/usr/bin/env python3
from _canonical_redirect import redirect_if_main as _redirect
_redirect(__name__, 'unfold_safe.py')
"""접힌 팔 펴기 v2 — FK로 안전한 연속 궤적을 계획해 한 번에 스트리밍한다.

1차 시도의 실패 원인 두 가지를 고쳤다:
  · 방향을 기하 가정(lift 먼저)으로 정했다 → 접힌 자세에선 lift+ 가 죠를 박는다
  · 관절별로 끊어 움직였다 → 웨이포인트를 연속 보간하고 전체 경로 z를 검증한다
"""
import json
import pathlib
import sys
import urllib.request

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import arm_lib

BASE = 'http://127.0.0.1:8765'
K = arm_lib.load_kinematics()
MP = arm_lib.load_mapping()
J = arm_lib.JOINTS

STEP = 8.0            # 한 걸음 [°]
Z_CLEAR = 0.02        # 이 z(구 책상면 +12cm)에 오르면 '떴다'로 본다
WORK = {'shoulder_pan': 0.0, 'shoulder_lift': -5.0, 'elbow_flex': 0.0,
        'wrist_flex': 88.0, 'wrist_roll': 0.0}          # 작업 자세 (POSES 상층 근방)

CAL = json.loads(pathlib.Path('~/.cache/huggingface/lerobot/calibration/robots/'
                              'so_follower/follower.json').expanduser().read_text())
BOUNDS = {j: (b[0] + 2.0, b[1] - 2.0)
          for j, b in arm_lib.calib_bounds(CAL).items() if j in J}


def post(op, **kw):
    r = urllib.request.Request(f'{BASE}/cmd', method='POST',
                               data=json.dumps(dict(op=op, **kw)).encode(),
                               headers={'Content-Type': 'application/json'})
    return json.load(urllib.request.urlopen(r, timeout=10))


def state():
    return json.load(urllib.request.urlopen(f'{BASE}/state', timeout=10))


def bail(msg):
    # 토크는 유지한다 (2026-08-25 전수 정비) — 서 있는 팔에서 토크 OFF 는 낙하다.
    post('stop')
    print(f'\n중단: {msg} — 정지(토크 유지). 팔은 자세를 지킵니다')
    sys.exit(1)


def fk_z(pos):
    q = arm_lib.servo_to_rad({f'{j}.pos': pos[j] for j in J}, MP)
    return K.fk_pos(q)[2] - arm_lib.PAN0[2]


def predict_z(pos, joint, delta):
    p = dict(pos); p[joint] = p[joint] + delta
    return fk_z(p)


def path_min_z(pos, joint, target, n=24):
    """현재 → 목표 **구간 전체**의 최저 죠 높이 [m].

    한 관절만 움직이는 구간이라 경로가 1차원이고, FK 로 촘촘히 훑으면 중간에
    죠가 얼마나 내려가는지 미리 알 수 있다. 이게 안전하면 8° 씩 쪼갤 이유가
    없다 — 쪼개면 걸음마다 1초 폴링이 붙어 펴는 데만 분 단위가 걸린다
    (2026-08-21 사용자 지시: "목표지점까지 부드럽게 한 번에").
    """
    lo = None
    for i in range(n + 1):
        p = dict(pos)
        p[joint] = pos[joint] + (target - pos[joint]) * i / n
        z = fk_z(p)
        lo = z if lo is None else min(lo, z)
    return lo


def main():
    st = state()
    if not (st['connected'] and st['calibrated'] and st['torque']):
        sys.exit('연결·캘리브·토크 ON 상태가 아닙니다')
    # 속도를 명시적으로 세운다 — 직전에 stop 이 있었으면 상한이 8(0.7°/s)로
    # 내려가 있고, goto 는 복원을 안 하므로 8° 걸음이 11초를 넘겨 자체
    # deadline(12초) 오탐으로 bail(토크 OFF) → 팔 낙하가 성립한다(감사 n3).
    post('speed', pct=90)   # 사용자: 추가 1.5배 — 걸음별 z 가드 있음
    pos = {j: st['pos'][j] for j in J}
    z = fk_z(pos)
    print(f'시작 z={z:+.4f}m · 자세 {({k: round(v,1) for k,v in pos.items()})}')

    # ── 계획 (2026-08-25 연속화): 종전 사다리 탐색을 **시뮬레이션**으로 돌려
    # 웨이포인트만 뽑고, smooth_move 가 15Hz pose 스트리밍으로 한 번에 흘린다 —
    # 관절별 [이동→정지→다음] 의 '끄덕끄덕' 제거. 기하 가드는 종전과 동일하고,
    # 실행 중에는 실측 z 하한·⛔·토크·추종지연을 0.5s 마다 감시한다.
    z0 = z
    waypoints = []
    wp_speed = []                      # 구간 속도 — 1단계(저공) 느리게, 2단계 빠르게
    # 1단계: 죠 부양 (시뮬레이션)
    for it in range(30):
        if z >= Z_CLEAR:
            break
        best = None
        for j in J:
            for d in (+STEP, -STEP):
                t = pos[j] + d
                if not (BOUNDS[j][0] <= t <= BOUNDS[j][1]):
                    continue
                gain = predict_z(pos, j, d) - z
                if best is None or gain > best[3]:
                    best = (j, d, t, gain)
        if best is None or best[3] < 0.002:
            bail(f'z 를 올릴 걸음이 없습니다 (z={z:+.4f})')
        j, d, t, gain = best
        pos = {**pos, j: t}
        z = z + gain
        waypoints.append(dict(pos))
        wp_speed.append(7.0)          # 책상 근접 — 접촉 마진이 얇다 (elbow 트립 실측)
    # 2단계: 작업 자세 (시뮬레이션 — 가드 동일)
    # ★ 순서는 팔꿈치 먼저가 맞다 (2026-08-26 되돌림). 어깨를 먼저 올리면
    # 접힌 팔의 죠가 **아래로 스윙**해 책상에 가까워진다(실측 z -0.024 하강,
    # 안전 검사에 걸림). 어깨의 추종 지연은 순서가 아니라 **감속(6°/s)과
    # 지연 판정 완화(정지했을 때만 걸림 + 상한 70°)** 로 푼다.
    order = ['elbow_flex', 'shoulder_lift', 'wrist_flex', 'shoulder_pan', 'wrist_roll']
    for j in order:
        if abs(pos[j] - WORK[j]) <= 1.5:
            continue
        zmin = path_min_z(pos, j, WORK[j])
        if zmin >= -0.02:
            pos = {**pos, j: WORK[j]}
            waypoints.append(dict(pos))
            # 중력을 드는 어깨는 느리게 — 명령이 앞서가면 지연으로 오탐된다
            wp_speed.append(6.0 if j == 'shoulder_lift' else 13.0)
        else:
            while abs(pos[j] - WORK[j]) > 1.5:
                d = max(-STEP, min(STEP, WORK[j] - pos[j]))
                t = pos[j] + d
                zp = predict_z(pos, j, d)
                if zp < -0.02:
                    bail(f'{j} 다음 걸음이 z={zp:+.4f} 로 내려감 — 순서 재검토 필요')
                pos = {**pos, j: t}
                waypoints.append(dict(pos))
                wp_speed.append(9.0)
        z = fk_z(pos)

    if not waypoints:
        print('이미 작업 자세 — 이동 생략')
        s = state()
        print(f"자세 {({k: round(v,1) for k,v in s['pos'].items()})}")
        return
    sys.path.insert(0, str(pathlib.Path.home() / 'so101-mobile-manipulation'))
    import smooth_move as sm
    start = {j: st['pos'][j] for j in J}
    ticks = sm.plan(start, waypoints, speeds=wp_speed)
    zmin_plan = sm.sweep_z(ticks)
    print(f'계획: 웨이포인트 {len(waypoints)} · 틱 {len(ticks)} · '
          f'경로 최저 z {zmin_plan:+.4f}')
    if zmin_plan < min(z0, -0.02) - 0.004:
        bail(f'계획 경로 z 위반 ({zmin_plan:+.4f}) — 실행 안 함')
    try:
        # ★ 하한은 **계획 경로 기준** (2026-08-26): 시작 높이 기준으로 잡으면
        # 공중에서 작업 자세로 내려오는 정상 이동이 차단된다(실측 트립).
        sm.stream(ticks, z_floor=min(z0, zmin_plan) - 0.012)
    except RuntimeError as e:
        bail(str(e))

    s = state()
    print(f'\n작업 자세 도달. z={fk_z(s["pos"]):+.4f}m · '
          f'자세 {({k: round(v,1) for k,v in s["pos"].items()})}')
    print('온도:', s.get('temp'), '· 전류:', s.get('current'))


if __name__ == '__main__':
    main()
