#!/usr/bin/env python3
"""죠를 천천히 내려 책상면 높이를 부하로 찾아 등록한다 — 파지 높이의 기준점.

## 왜 비전이 아니라 접촉인가

단안 손목캠에는 깊이가 없다. 면적으로 근접을 어림할 수는 있지만 물체마다 크기가
달라 절대 높이가 안 나온다. 반면 책상면은 **한 번 재면 변하지 않는 상수**라,
접촉으로 등록해 두고 이후에는 "바닥 + 물체 반높이" 로 계산하면 된다.
(딥리서치 확인: JHU 픽앤플레이스가 심을 물려 접촉 등록 후 고정 오프셋만 명령해
 파지 100% 를 얻었다.)

## 안전

- 한 스텝 2mm, 매 스텝 부하를 읽어 임계를 넘으면 즉시 정지하고 5mm 후퇴한다.
- Torque_Limit 을 평소(600)보다 낮춰(350) 접촉해도 책상·죠를 밀지 않게 한다.
- 임계는 무부하 기준선의 상대값으로 잡는다 — 자세마다 중력 부하가 다르기 때문이다.

사용: python3 probe_floor.py [x] [y]      기본 0.20 0.00
"""
import json
import pathlib
import sys
import time

import glob

HERE = pathlib.Path(__file__).parent
Z_START = -0.05          # 여기서부터 내려간다 (2026-08-19 실측: 새 좌표계에서
                         # 죠가 책상에 닿은 안착 자세의 TCP z ≈ -0.078 — 3cm 위)
Z_MIN = -0.20            # 이보다 내려가면 중단 (안전)
STEP = 0.002             # 한 스텝 2mm
LOAD_MARGIN = 200        # 기준선 대비 이만큼 오르면 접촉
BACKOFF = 0.005          # 접촉 후 후퇴량


def main():
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.20
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.00

    sys.path.insert(0, str(HERE))
    import arm_lib
    K = arm_lib.load_kinematics()
    import math
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode

    port = sorted(glob.glob('/dev/ttyACM*'))[0]
    ARM = arm_lib.JOINTS
    motors = {j: Motor(i + 1, 'sts3215', MotorNormMode.DEGREES) for i, j in enumerate(ARM)}
    motors['gripper'] = Motor(6, 'sts3215', MotorNormMode.RANGE_0_100)
    cp = pathlib.Path.home() / '.cache/huggingface/lerobot/calibration/robots/so_follower/follower.json'
    cal = {k: MotorCalibration(**v) for k, v in json.loads(cp.read_text()).items()}
    bus = FeetechMotorsBus(port=port, motors=motors, calibration=cal)
    try:
        bus._connect(handshake=False)
    except AttributeError:
        bus.connect(handshake=False)
    bus.write_calibration(cal)

    mapping = arm_lib.load_mapping()

    def goto(z):
        bf = tuple(p + o for p, o in zip((x, y, z), arm_lib.PAN0))
        q = K.ik_best(*bf, pitch=math.radians(-90))
        if q is None:
            return False
        # rad_to_servo 는 lerobot 액션 키('joint.pos')를 주지만 bus 는 모터명을 받는다
        want = {k.replace('.pos', ''): v
                for k, v in arm_lib.rad_to_servo(q, mapping).items()}
        bus.sync_write('Goal_Position', want)
        return True

    def load():
        v = bus.sync_read('Present_Load', ARM, normalize=False)
        return sum(abs(int(a)) for a in v.values())

    print(f'접촉 탐지 · x={x:.2f} y={y:.2f} · 스텝 {STEP*1000:.0f}mm '
          f'· 임계 기준선+{LOAD_MARGIN}\n')
    bus.disable_torque()
    for m in list(motors):
        bus.write('Maximum_Velocity_Limit', m, 254, normalize=False)
    bus.sync_write('Goal_Velocity', {m: 40 for m in motors}, normalize=False)   # 천천히
    bus.sync_write('Acceleration', {m: 10 for m in motors}, normalize=False)
    bus.sync_write('Torque_Limit', {m: 350 for m in motors}, normalize=False)   # 약하게
    # ★ 켜기 전에 목표를 현재 위치(raw)로 덮는다 — 이전 목표가 남아 있으면 토크가
    # 들어가는 순간 그리로 튄다 (2026-08-19 교훈, arm_gui._do_torque 와 동일).
    raw = bus.sync_read('Present_Position', normalize=False)
    bus.sync_write('Goal_Position', raw, normalize=False)
    for m in list(motors):
        bus.enable_torque(m)
        time.sleep(0.12)

    try:
        goto(Z_START)
        time.sleep(8.0)                      # 안착 자세에서 오는 첫 이동이 가장 길다
        base = min(load() for _ in [time.sleep(0.15) or 0 for _ in range(6)])
        print(f'무부하 기준선 {base}\n')

        z = Z_START
        hit = None
        while z > Z_MIN:
            z -= STEP
            if not goto(z):
                print(f'z={z:+.3f} IK 해 없음 — 중단'); break
            time.sleep(0.55)
            lo = load()
            mark = ''
            if lo > base + LOAD_MARGIN:
                mark = '  ← 접촉'
                hit = z
            print(f'  z={z:+.3f}  부하 {lo:5d} (기준선 +{lo-base:4d}){mark}')
            if hit is not None:
                break

        if hit is None:
            print('\n접촉 없음 — Z_MIN 까지 내려갔습니다. 물체·책상 위치를 확인하세요')
            rc = 1
        else:
            print(f'\n접촉 z={hit:+.4f}m → {BACKOFF*1000:.0f}mm 후퇴')
            goto(hit + BACKOFF)
            time.sleep(2.5)
            p = HERE / 'servo_gain.json'
            d = json.loads(p.read_text())
            d['floor_z_m'] = round(hit, 4)
            d['floor_note'] = (f'{time.strftime("%Y-%m-%d")} 접촉 실측 (x={x:.2f}, y={y:.2f}). '
                               'pan 축 기준 책상면 높이. 파지 높이 = floor_z + 물체 반높이 + 여유. '
                               '로봇 베이스나 책상을 옮기거나 재캘리브레이션하면 다시 잴 것.')
            # 재측정했으므로 stale 표시를 지운다 — 남겨 두면 load_gain 가드가
            # 새 값까지 계속 막는다. (stale_* 딕셔너리가 무효 목록의 원본이다)
            for k in [k for k in d if k.startswith('stale_') and isinstance(d[k], dict)]:
                d[k].pop('floor_z_m', None)
            p.write_text(json.dumps(d, ensure_ascii=False, indent=2) + '\n')
            print(f'저장 → floor_z_m = {hit:.4f} (stale 표시 해제)')
            rc = 0
    finally:
        # 크래시로 나가도 토크 한도는 원복하고 곱게 놓는다 — disconnect 가
        # 토크를 내리므로 팔은 그 자리에서 천천히 내려앉는다 (탐지 높이라 안전)
        try:
            bus.sync_write('Torque_Limit', {m: 600 for m in motors}, normalize=False)
        except Exception:
            pass
        bus.disconnect()
    return rc


if __name__ == '__main__':
    sys.exit(main())
