#!/usr/bin/env python3
"""파지 데모 — 뎁스캠이 본 빨간 물체를 광선∩책상평면으로 위치 추정해 잡는다.

pick_red.py(손목캠 폐루프)와 달리 **hand-eye 정합**을 쓰는 첫 소비자다:
    시선 광선: origin = t,  dir = R·[bx, by, 1]   (handeye.json)
    물체 중심: 광선 ∩ 평면 z = floor + h_center   (물체 깊이 불필요 — 고무도 됨)

사용:
    python3 pick_demo.py standing    # 서 있는 체스말 (중심 3.5cm, 파지 4.5cm 높이)
    python3 pick_demo.py lying       # 누운 체스말   (중심 1.1cm, 파지 1.2cm 높이)
    python3 pick_demo.py standing --dry   # 위치 계산·프리플라이트만, 이동 없음

안전: 팔 전체 토크 ON 필요. 이동은 서버 ik(스톨·과전류 감시 내장) 경유, 매 지점
도달·토크 확인. 관측 실패는 이동 실패가 아니다 — stop 없이 중단(감사 M1).
"""
import argparse
import json
import math
import pathlib
import sys
import time
import urllib.request

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import arm_lib

BASE = 'http://127.0.0.1:8765'
J = arm_lib.JOINTS
POSE = {                    # (블롭 중심 높이, 파지 TCP 높이) — floor 기준 [m]
    'standing': (0.035, 0.045),
    'lying':    (0.011, 0.008),   # 죠 끝이 몸통 중심선(11mm) **아래**로 내려가야
                                  # 물체가 입 안에 들어온다 — 16mm 는 바깥턱에
                                  # 닿았다(실물 1차 실패). 최종 하강은 15% 저속
}
APPROACH_CAND = (0.02, 0.005, -0.01)   # 접근 고도 후보 — 원거리 x 는 높은 z 가
LIFT_CAND = (0.03, 0.015, 0.0)         # 안 풀린다(리치). IK 되는 첫 값을 쓴다
GRIP_OPEN_ABS = 55          # 절대 개방각 — delta 방식은 이미 열린 상태에서 이중
GRIP_CLOSE_ABS = 1          # 개방(99, 아랫턱 젖힘)을 만들었다(실측). 절대각으로만.
# 죠 닫힘축 실측(2026-08-20, MuJoCo 두 손끝 사이트 — 실물 대조된 롤 오프셋
# 반영): 닫힘축 yaw = pan + CLOSE_AXIS + roll [°]. 롤 0 에서 닫힘축은 방사
# 방향과 거의 직교(-94.3)라 방사로 누운 물체가 잡혔다. 방향 파지는 물체
# 장축과 닫힘축이 직교하도록 roll 을 푼다 (축은 180° 대칭 → ±90 접기).
CLOSE_AXIS = -94.3


def post(op, **kw):
    r = urllib.request.Request(f'{BASE}/cmd', method='POST',
                               data=json.dumps(dict(op=op, **kw)).encode(),
                               headers={'Content-Type': 'application/json'})
    return json.load(urllib.request.urlopen(r, timeout=15))


def get(path):
    return json.loads(urllib.request.urlopen(f'{BASE}{path}', timeout=15).read())


def bail(msg):
    post('stop')            # ARM 목표만 현재로 — 그리퍼 예압 유지
    print(f'중단: {msg} — 정지(토크 유지)')
    sys.exit(1)


def read_bearing(tries=10, need=5, with_axis=False):
    """빨간 블롭의 픽셀 방위각 중앙값 (+옵션: 주축 각·fx·fy). 깊이 불필요."""
    pts, axes, fxy = [], [], None
    for _ in range(tries):
        b = get('/blob').get('blob') or {}
        if b.get('u') is not None and b.get('fx'):
            pts.append([(b['u'] - b['w'] / 2) / b['fx'],
                        (b['v'] - b['h'] / 2) / b['fy']])
            fxy = (b['fx'], b['fy'])
            if b.get('axis_deg') is not None:
                axes.append(math.radians(b['axis_deg']))
        time.sleep(0.2)
    if len(pts) < need:
        return (None, None, None) if with_axis else None
    a = np.array(pts)
    med = np.median(a, axis=0)
    keep = a[np.linalg.norm(a - med, axis=1) < 0.008]
    brg = keep.mean(axis=0) if len(keep) >= need else None
    if not with_axis:
        return brg
    axis = None
    if brg is not None and len(axes) >= need:
        zc = np.mean([np.exp(2j * ang) for ang in axes])  # 180° 대칭 원형 평균
        if abs(zc) > 0.7:                                 # 표본 일관성 게이트
            axis = math.degrees(np.angle(zc) / 2)
    return brg, axis, fxy


def ray_plane(brg, R, t, floor, h_center):
    """방위각 → 광선 ∩ 평면(z = floor + h_center) 교점 (x, y)."""
    d = R @ np.array([brg[0], brg[1], 1.0])
    if abs(d[2]) < 1e-6:
        return None
    s = (floor + h_center - t[2]) / d[2]
    if not (0.2 < s < 1.5):
        return None
    p = t + s * d
    return float(p[0]), float(p[1])


def locate(R, t, floor, h_center):
    brg = read_bearing()
    if brg is None:
        return None
    return ray_plane(brg, R, t, floor, h_center)


def piece_yaw(brg, axis_img_deg, fxy, R, t, floor, h_center):
    """이미지 주축을 책상 평면에 투영해 물체 장축의 로봇좌표 yaw[°]를 얻는다.

    축 위 두 픽셀(40px 간격)을 각각 광선∩평면으로 내려 잇는다 — 카메라
    기울기·투영 왜곡이 자동으로 반영된다(이미지 각도를 그대로 쓰면 틀린다)."""
    a = math.radians(axis_img_deg)
    d_brg = np.array([math.cos(a) / fxy[0], math.sin(a) / fxy[1]]) * 40.0
    p1 = ray_plane(np.asarray(brg), R, t, floor, h_center)
    p2 = ray_plane(np.asarray(brg) + d_brg, R, t, floor, h_center)
    if p1 is None or p2 is None:
        return None
    return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))


def wait_gripper_settle(timeout=35.0):
    """그리퍼가 멈출 때까지 대기 — 고정 sleep 은 닫힘(~15s)을 못 기다려
    물체를 덜 문 채 들어올렸다(실물 1차 실패). 20초는 저속 프로파일의 전개방
    (55°, ~20초)을 못 기다려 이동 중 판정이 났다(2026-08-20 데모 실측) — 35초."""
    prev = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        time.sleep(1.2)
        g = get('/state')['pos'].get('gripper')
        if prev is not None and g is not None and abs(g - prev) < 0.3:
            return g
        prev = g
    return prev


def move_and_wait(x, y, z, timeout=25.0, roll=None):
    pre = get('/state').get('log', [])
    pre_tail = pre[-1] if pre else ''            # 이 이동 전의 마지막 로그
    kw = dict(x=round(x, 4), y=round(y, 4), z=round(z, 4), pitch=-90)
    if roll is not None:                         # 방향 파지 — 하강 중 롤 유지
        kw['roll'] = round(roll, 1)
    r = post('ik', **kw)
    if not r.get('ok'):
        bail(f'IK 실패 ({x:.3f},{y:.3f},{z:.3f}): {r.get("msg")}')
    mapping = arm_lib.load_mapping()
    want = {k.replace('.pos', ''): v
            for k, v in arm_lib.rad_to_servo(r['q'], mapping).items()}
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        st = get('/state')
        tail = st.get('log', [])[-1] if st.get('log') else ''
        if '⛔' in tail:
            bail(f'서버 거부/안전장치: {tail}')       # 게이트 거부는 토크 유지라 여기서 잡는다 (m47)
        if not (st.get('torque') and st.get('connected')):
            print(f'중단: 서버 안전장치 발동/통신 이상 — {tail}')
            sys.exit(1)
        gap = max(abs((st['pos'][j] - want[j] + 180) % 360 - 180) for j in J)
        if gap < 1.5:   # 실측: shoulder_lift 중력 정착 오차 1.2° — 0.8은 도달 불가.
                        # 파지 여유는 h_grip +4mm(m50)로 확보
            return
        # 서버 완료 신호 (14차 리뷰 M5, 2026-08-20 실측 2회): 서버 도달 기준은
        # 3.0° 라 1.5~3.0° 정착은 gap 만으론 영영 미도달 → 타임아웃(리치 경계
        # 상승에서 재현). 이 이동이 만든 **새** '이동 완료' 로그 + gap<3.5° 면
        # 도달로 본다. 직전 이동과 로그 문자열이 우연히 같으면(전류피크 동일)
        # 신호를 놓치지만, 그때는 기존 타임아웃 경로로 떨어질 뿐이다(fail-safe).
        if '이동 완료' in tail and tail != pre_tail and gap < 3.5:
            return
        time.sleep(0.3)
    bail(f'도달 시간 초과 ({x:.3f},{y:.3f},{z:.3f})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pose', choices=list(POSE))
    ap.add_argument('--dry', action='store_true', help='위치 계산·검증만, 이동 없음')
    a = ap.parse_args()
    h_center, h_grip = POSE[a.pose]

    he_p = pathlib.Path(__file__).parent / 'handeye.json'
    if not he_p.exists():
        sys.exit('handeye.json 없음 — 정합부터')
    he = json.loads(he_p.read_text())
    R, t = np.array(he['R']), np.array(he['t'])
    floor = arm_lib.load_gain('floor_z_m')['floor_z_m']
    z_grip = floor + h_grip
    # 파지 오프셋 — 교시 상수를 필수 선언으로 로드: stale/누락이면 여기서 멈춘다
    # (리뷰 M3: 무선언 .get(기본 [0,0]) 은 12mm 보정이 말없이 사라지는 경로였다)
    OFF = arm_lib.load_gain('grasp_xy_offset_m')['grasp_xy_offset_m']
    # TODO(2026-08-20 사용자 피드백): 이 오프셋으로 파지는 성공했으나 하강 중
    # 죠가 물체를 살짝 스쳤다 — "아랫턱이 물체보다 조금 더 왼쪽에 온 상태로
    # 파지돼도 된다". 다음 실물 세션에서 손목캠으로 방향 확인 후 수 mm 보정.

    brg, axis_img, fxy = read_bearing(with_axis=True)
    loc = ray_plane(brg, R, t, floor, h_center) if brg is not None else None
    if loc is None:
        sys.exit('물체 미검출 — 시야·조명 확인 (이동 안 함)')
    x, y = loc[0] - OFF[0], loc[1] - OFF[1]

    # 방향 파지 (2026-08-20): 물체 장축 yaw 를 구해 죠 닫힘축이 직교하도록
    # 손목 롤을 푼다. 축이 안 잡히면(원형 블롭·표본 불일치) 종전대로 롤 없음.
    yaw = (piece_yaw(brg, axis_img, fxy, R, t, floor, h_center)
           if axis_img is not None else None)

    def roll_for(yaw_deg, tx, ty):
        v = yaw_deg + 90.0 - math.degrees(math.atan2(ty, tx)) - CLOSE_AXIS
        return ((v + 90.0) % 180.0) - 90.0

    roll = roll_for(yaw, x, y) if yaw is not None else None
    print(f'물체({a.pose}) 추정: ({loc[0]:+.3f}, {loc[1]:+.3f}) '
          f'→ 오프셋 보정 ({x:+.3f}, {y:+.3f}) · 파지 z {z_grip:+.3f}')
    print(f'   장축 yaw: {"%.1f°" % yaw if yaw is not None else "불명(원형/불일치)"}'
          f' → 손목 롤 {"%.1f°" % roll if roll is not None else "0 (기본)"}')

    # 접근·상승 고도 적응 선택 + 프리플라이트 (이동 전)
    K = arm_lib.load_kinematics()

    def feasible_z(cands):
        for z in cands:
            bf = tuple(p + o for p, o in zip((x, y, z), arm_lib.PAN0))
            if K.ik_best(*bf, pitch=math.radians(-90)) is not None:
                return z
        return None
    APPROACH_Z = feasible_z(APPROACH_CAND)
    LIFT_Z = feasible_z(LIFT_CAND)
    if APPROACH_Z is None or LIFT_Z is None:
        sys.exit(f'접근/상승 고도 후보가 전부 IK 불가 ({x:+.3f},{y:+.3f}) — 물체가 '
                 f'리치 경계 밖입니다 (이동 안 함)')
    print(f'   접근 z {APPROACH_Z:+.3f} · 상승 z {LIFT_Z:+.3f} (적응 선택)')
    if not (0.10 <= x <= 0.28 and abs(y) <= 0.12):     # 가장 싼 검사 먼저
        sys.exit(f'추정 위치가 작업 영역 밖 ({x:+.3f},{y:+.3f}) — 이동 안 함')
    # 캘리브 범위 검사 — 이동 후 타임아웃이 아니라 이동 전에 잡는다 (m48)
    mp = arm_lib.load_mapping()
    import json as _json
    cal = _json.loads((pathlib.Path.home() / '.cache/huggingface/lerobot/'
                       'calibration/robots/so_follower/follower.json').read_text())
    bounds = arm_lib.calib_bounds(cal)
    if roll is not None and not (bounds['wrist_roll'][0] + 2 <= roll
                                 <= bounds['wrist_roll'][1] - 2):
        sys.exit(f'목표 롤 {roll:+.1f}° 가 캘리브 범위 밖 (이동 안 함)')
    for tag, (px, py, pz) in (('접근', (x, y, APPROACH_Z)),
                              ('파지', (x, y, z_grip)),
                              ('상승', (x, y, LIFT_Z))):
        bf = tuple(p + o for p, o in zip((px, py, pz), arm_lib.PAN0))
        q = K.ik_best(*bf, pitch=math.radians(-90))
        if q is None:
            sys.exit(f'{tag} 지점 ({px:+.3f},{py:+.3f},{pz:+.3f}) IK 해 없음 — '
                     f'물체가 작업 범위 밖입니다 (이동 안 함)')
        for i, jn in enumerate(J):
            v = mp['signs'][jn] * math.degrees(q[i]) + mp['offsets'][jn]
            if not (bounds[jn][0] + 2 <= v <= bounds[jn][1] - 2):
                sys.exit(f'{tag} 지점의 {jn}={v:+.1f}° 가 캘리브 범위 밖 (이동 안 함)')
    if a.dry:
        print('--dry: 검증 통과, 이동 없음')
        return

    st = get('/state')
    if not (st['connected'] and st['calibrated'] and st['torque']):
        sys.exit('연결·캘리브·토크 ON 후 실행하세요 (팔이 접혀 있으면 unfold_safe 먼저)')

    post('speed', pct=40)   # 극저속 계단 떨림 방지 — 자유공간 이동은 40%
    print('① 접근 자세로 이동')
    move_and_wait(x, y, APPROACH_Z, roll=roll)
    print('② 그리퍼 개방')
    g_now = get('/state')['pos'].get('gripper', 50)
    post('goto', joint='gripper', value=round(g_now, 1))  # 위치 재전송 = 과부하 보호 해제
    time.sleep(1.0)
    post('goto', joint='gripper', value=GRIP_OPEN_ABS)
    wait_gripper_settle()
    # 재관측(re-look) — 접근 자세에서 팔이 시야를 바꿨을 수 있어 한 번 갱신
    loc2 = locate(R, t, floor, h_center)
    d2 = (math.hypot(loc2[0] - (x + OFF[0]), loc2[1] - (y + OFF[1]))
          if loc2 else None)
    if d2 is None:
        print('   재관측 실패 — 최초 추정 유지')
    elif d2 < 0.015:
        x, y = loc2[0] - OFF[0], loc2[1] - OFF[1]
        print(f'   재관측 보정 → ({x:+.3f}, {y:+.3f})')
    elif d2 < 0.03:
        x, y = loc2[0] - OFF[0], loc2[1] - OFF[1]
        print(f'   ⚠ 재관측 보정 {1000*d2:.0f}mm — 큽니다. 하강을 지켜보세요')
    else:
        bail(f'재관측이 {1000*d2:.0f}mm 어긋남 — 물체가 움직였거나 오검출 (m49)')
    if yaw is not None:
        roll = roll_for(yaw, x, y)     # 재관측으로 pan 이 바뀌었을 수 있다
    print('③ 하강 (2단 — 최종은 15% 저속)')
    move_and_wait(x, y, (APPROACH_Z + z_grip) / 2, roll=roll)
    post('speed', pct=15)
    move_and_wait(x, y, z_grip, timeout=35.0, roll=roll)
    print('④ 파지')
    post('goto', joint='gripper', value=GRIP_CLOSE_ABS)
    g = wait_gripper_settle()
    if g is None:                      # 읽기 실패 — 압착 목표(1)를 남긴 채
        g = get('/state')['pos'].get('gripper')   # 진행하면 안 된다 (리뷰 M7)
    if g is None:
        bail('그리퍼 상태 읽기 실패 — 압착 목표가 남아 있습니다. 패널에서 '
             '그리퍼 목표를 현재값으로 재전송해 압력을 해제하세요')
    # ★ 압력 해제 — 목표를 현재값으로 되쓴다. 목표 0 을 남기면 접촉 후에도
    # 계속 쥐어짜 수 분 뒤 펌웨어 과부하 보호(25%)가 떠서 열기가 거부된다
    # (실측 2026-08-20: RxPacketError Overload). 위치 유지 토크만으로 충분.
    post('goto', joint='gripper', value=round(g, 1))
    post('speed', pct=40)
    print(f'   그리퍼 {g:.1f} 에서 닫힘 완료 (0 근처면 헛집음)')
    print('⑤ 들어올리기')
    move_and_wait(x, y, LIFT_Z, roll=roll)
    # 검증: 팔과 함께 블롭이 움직이는가 (pan 흔들기)
    b0 = read_bearing()
    post('jog', joint='shoulder_pan', delta=6)
    time.sleep(4)
    b1 = read_bearing()
    post('jog', joint='shoulder_pan', delta=-6)
    time.sleep(4)
    if b0 is not None and b1 is not None:
        moved = float(np.linalg.norm(np.array(b1) - np.array(b0))) * 0.6
        verdict = '물었음 (블롭이 팔을 따라옴)' if moved > 0.015 else \
                  '놓친 듯 (블롭이 제자리)'
        print(f'⑥ 판정: {verdict} — 시선 이동 {1000*moved:.0f}mm 상당, 그리퍼 {g:.1f}')
    else:
        print('⑥ 판정 불가(관측 부족) — 눈으로 확인해 주세요')
    print('완료 — 팔은 물체를 든 채 대기 (토크 ON). 내려놓기/파킹은 별도 지시로.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        try:
            post('stop')
        except Exception:
            pass
        sys.exit('\n사용자 중단 — 정지(토크 유지)')
    except Exception:
        try:
            post('stop')
        except Exception:
            pass
        raise
