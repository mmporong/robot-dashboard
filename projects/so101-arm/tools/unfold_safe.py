#!/usr/bin/env python3
"""접힌 팔 펴기 v2 — 가정 대신 계산: 매 단계 FK 자코비안으로 '죠가 올라가는'
관절·방향을 고르고, 실측 z 변화가 예측과 어긋나면(걸림) 즉시 중단한다.

1차 시도의 실패 원인 두 가지를 고쳤다:
  · 방향을 기하 가정(lift 먼저)으로 정했다 → 접힌 자세에선 lift+ 가 죠를 박는다
  · 위치 정체만 감시했다 → 예측 z 와 실측 z 의 괴리(부분 걸림)도 함께 본다
"""
import json
import pathlib
import sys
import time
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


def move_step(joint, tgt, expect_z, timeout=None):
    """goto — 1초 폴링으로 추종·z 를 감시한다. timeout 은 이동량에 비례."""
    r = post('goto', joint=joint, value=round(tgt, 2))
    if not r.get('ok'):
        bail(f'{joint} goto 요청 실패 {r}')
    slow, prev = 0, None
    deadline = time.monotonic() + (timeout if timeout else 12.0)
    while True:
        time.sleep(1.0)
        s = state()
        tail = s['log'][-1] if s['log'] else ''
        if '⛔' in tail or '거부' in tail:
            bail(f'{joint} 거부: {tail}')
        if not s['torque']:
            bail(f'이동 중 토크 낙하: {tail}')
        now = s['pos'][joint]
        gap = abs(now - tgt)
        # 2.5°: P게인 16 의 정지 오차(실측 1.8~1.9°, 2026-08-25 접힘 팔꿈치)
        # 밖으로. 정확도는 다음 걸음의 절대 목표가 회복한다.
        if gap < 2.5:
            z = fk_z(s['pos'])
            return s['pos'], z
        slow = slow + 1 if (prev is not None and abs(now - prev) < 0.3) else 0
        prev = now
        if slow >= 2 and gap > 3.0:
            bail(f'{joint} 추종 실패 — {gap:.1f}° 남기고 정체 (걸림 의심)')
        if time.monotonic() > deadline:
            bail(f'{joint} 시간 초과 — {gap:.1f}° 남음')


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
    # 2단계: 작업 자세 (시뮬레이션 — 가드 동일)
    order = ['elbow_flex', 'shoulder_lift', 'wrist_flex', 'shoulder_pan', 'wrist_roll']
    for j in order:
        if abs(pos[j] - WORK[j]) <= 1.5:
            continue
        zmin = path_min_z(pos, j, WORK[j])
        if zmin >= -0.02:
            pos = {**pos, j: WORK[j]}
            waypoints.append(dict(pos))
        else:
            while abs(pos[j] - WORK[j]) > 1.5:
                d = max(-STEP, min(STEP, WORK[j] - pos[j]))
                t = pos[j] + d
                zp = predict_z(pos, j, d)
                if zp < -0.02:
                    bail(f'{j} 다음 걸음이 z={zp:+.4f} 로 내려감 — 순서 재검토 필요')
                pos = {**pos, j: t}
                waypoints.append(dict(pos))
        z = fk_z(pos)

    if not waypoints:
        print('이미 작업 자세 — 이동 생략')
        s = state()
        print(f"자세 {({k: round(v,1) for k,v in s['pos'].items()})}")
        return
    sys.path.insert(0, str(pathlib.Path.home() / 'so101_tools'))
    import smooth_move as sm
    start = {j: st['pos'][j] for j in J}
    ticks = sm.plan(start, waypoints, speed_dps=30.0)
    zmin_plan = sm.sweep_z(ticks)
    print(f'계획: 웨이포인트 {len(waypoints)} · 틱 {len(ticks)} · '
          f'경로 최저 z {zmin_plan:+.4f}')
    if zmin_plan < min(z0, -0.02) - 0.004:
        bail(f'계획 경로 z 위반 ({zmin_plan:+.4f}) — 실행 안 함')
    try:
        sm.stream(ticks, z_floor=z0 - 0.012)
    except RuntimeError as e:
        post('stop')
        bail(str(e))

    s = state()
    print(f'\n작업 자세 도달. z={fk_z(s["pos"]):+.4f}m · '
          f'자세 {({k: round(v,1) for k,v in s["pos"].items()})}')
    print('온도:', s.get('temp'), '· 전류:', s.get('current'))


if __name__ == '__main__':
    main()
