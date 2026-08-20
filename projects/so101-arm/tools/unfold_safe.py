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
    post('stop')
    post('torque', on=False)
    print(f'\n중단: {msg} — 정지 + 토크 OFF')
    sys.exit(1)


def fk_z(pos):
    q = arm_lib.servo_to_rad({f'{j}.pos': pos[j] for j in J}, MP)
    return K.fk_pos(q)[2] - arm_lib.PAN0[2]


def predict_z(pos, joint, delta):
    p = dict(pos); p[joint] = p[joint] + delta
    return fk_z(p)


def move_step(joint, tgt, expect_z):
    """goto 한 걸음 — 1초 폴링으로 추종·z 를 감시한다."""
    r = post('goto', joint=joint, value=round(tgt, 2))
    if not r.get('ok'):
        bail(f'{joint} goto 요청 실패 {r}')
    slow, prev = 0, None
    deadline = time.monotonic() + 12.0
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
        if gap < 1.5:
            z = fk_z(s['pos'])
            return s['pos'], z
        slow = slow + 1 if (prev is not None and abs(now - prev) < 0.3) else 0
        prev = now
        if slow >= 2 and gap > 2.0:
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
    post('speed', pct=25)   # 15%는 극저속 계단 떨림(2026-08-20) — 자유공간이라 25%
    pos = {j: st['pos'][j] for j in J}
    z = fk_z(pos)
    print(f'시작 z={z:+.4f}m · 자세 {({k: round(v,1) for k,v in pos.items()})}')

    # ── 1단계: 죠 띄우기 — 매 걸음 z 를 가장 올리는 관절·방향 선택 ──────────
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
        pos2, z2 = move_step(j, t, z + gain)
        made = z2 - z
        print(f'  [{it+1:2d}] {j} {d:+.0f}° → z {z:+.4f}→{z2:+.4f} '
              f'(예측 {gain*1000:+.1f}mm · 실측 {made*1000:+.1f}mm)')
        if made < gain * 0.4 - 0.001:
            bail(f'{j} 이동이 예측만큼 z 를 못 올림 — 걸림 의심')
        pos, z = pos2, z2
        time.sleep(0.5)
    print(f'죠 부양 완료: z={z:+.4f}m\n')

    # ── 2단계: 작업 자세로 — 관절별 소걸음, z 가 -0.02 밑으로 떨어지면 중단 ──
    order = ['elbow_flex', 'shoulder_lift', 'wrist_flex', 'shoulder_pan', 'wrist_roll']
    for j in order:
        while abs(pos[j] - WORK[j]) > 1.5:
            d = max(-STEP, min(STEP, WORK[j] - pos[j]))
            t = pos[j] + d
            zp = predict_z(pos, j, d)
            if zp < -0.02:
                bail(f'{j} 다음 걸음이 z={zp:+.4f} 로 내려감 — 순서 재검토 필요')
            pos, z = move_step(j, t, zp)
            print(f'  {j:14s} → {pos[j]:+7.1f}°  (z={z:+.4f})')
            time.sleep(0.3)

    s = state()
    print(f'\n작업 자세 도달. z={fk_z(s["pos"]):+.4f}m · '
          f'자세 {({k: round(v,1) for k,v in s["pos"].items()})}')
    print('온도:', s.get('temp'), '· 전류:', s.get('current'))


if __name__ == '__main__':
    main()
