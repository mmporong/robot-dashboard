#!/usr/bin/env python
"""SO-101 실팔 MuJoCo 미러 (rlwalk 파이썬 — mujoco 3.11).

패널 서버(8765)의 /state 를 10Hz 로 읽어(읽기 전용 — 팔 명령은 일절 없음)
MJCF qpos 에 반영하고, 뎁스캠 /blob 이 보는 빨간 물체를 체스말 프록시로
배치한다. 좌표 변환은 frame_fit.py 가 적합한 sim_frame.json (RMS 0.00mm,
p_sim = R @ p_K + t · qpos = URDF q 직결).

사용:
  뷰어(실시간 미러):  ~/miniforge3/envs/rlwalk/bin/python sim_view.py
  정지 자세 뷰어:     ... sim_view.py --deg "shoulder_pan=-6.3,shoulder_lift=-2.2,elbow_flex=0.9,wrist_flex=88.1,wrist_roll=0,gripper=2.6"
  스냅샷(무화면):     ... sim_view.py --deg "..." --snapshot out.png [--cam wrist_cam]
  물체 수동 배치:     --piece-at "0.19,0.02" --piece lying|standing
"""
import argparse
import json
import math
import pathlib
import sys
import time
import urllib.request

import numpy as np

D = pathlib.Path(__file__).parent
sys.path.insert(0, str(D.parent))
import arm_lib

BASE = 'http://127.0.0.1:8765'
JN = arm_lib.JOINTS                       # 5관절 (gripper 별도)
LYING_QUAT = (0.7071068, 0.0, 0.7071068, 0.0)   # 원기둥 z축 → x축 (누움)
# 실물 그리퍼는 URDF 대비 롤이 180° 돌아 조립돼 있다(움직이는 턱이 실물은
# 아래·모델은 위, 2026-08-20 실물 대조). 캘리브 오프셋이 이를 흡수해 TCP
# 위치 적합(롤 축 위의 점)에는 안 드러난다 — 표시용 오프셋으로 보정.
ROLL_OFFSET_RAD = math.pi
GRIP_HOLD_DEG = 25                        # 이보다 닫혀 있으면 물체를 문 것으로 표시


def get(path, timeout=2.0):
    return json.loads(urllib.request.urlopen(f'{BASE}{path}', timeout=timeout).read())


def load_frame():
    f = json.loads((D / 'sim_frame.json').read_text())
    assert f['qpos_mode'] == 'urdf_q', f'미검증 qpos 모드: {f["qpos_mode"]}'
    return np.array(f['R']), np.array(f['t'])


def panel_to_sim(p, R, t):
    """패널 좌표(PAN0 기준) → 시뮬 월드."""
    return R @ (np.array(p, float) + np.array(arm_lib.PAN0)) + t


def read_blob_xy(R_he, t_he, floor, h_center):
    """뎁스캠 방위각 광선 ∩ 평면 — pick_demo.locate 와 같은 식 (1샷)."""
    b = get('/blob').get('blob') or {}
    if b.get('u') is None or not b.get('fx'):
        return None
    d = R_he @ np.array([(b['u'] - b['w'] / 2) / b['fx'],
                         (b['v'] - b['h'] / 2) / b['fy'], 1.0])
    if abs(d[2]) < 1e-6:
        return None
    s = (floor + h_center - t_he[2]) / d[2]
    if not (0.2 < s < 1.5):
        return None
    p = t_he + s * d
    return float(p[0]), float(p[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--deg', help='정지 자세 "관절=도,..." (서버 대신)')
    ap.add_argument('--snapshot', help='무화면 렌더 → PNG 경로 (뷰어 없이 종료)')
    ap.add_argument('--record', help='무화면 녹화 — 프레임 PNG 디렉터리 (라이브 상태 추적)')
    ap.add_argument('--fps', type=float, default=10.0, help='--record 프레임률')
    ap.add_argument('--seconds', type=float, default=90.0, help='--record 길이')
    ap.add_argument('--cam', default='', help='스냅샷/녹화 카메라명 (기본 자유 시점)')
    ap.add_argument('--piece', choices=['lying', 'standing'], default='lying')
    ap.add_argument('--piece-at', help='물체 패널 좌표 "x,y" 수동 지정')
    ap.add_argument('--hz', type=float, default=10.0)
    a = ap.parse_args()

    import mujoco
    model = mujoco.MjModel.from_xml_path(str(D / 'scene_mirror.xml'))
    data = mujoco.MjData(model)
    R, t = load_frame()
    MP = arm_lib.load_mapping()
    floor = arm_lib.load_gain('floor_z_m')['floor_z_m']

    jadr = {j: model.jnt_qposadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JN + ['gripper']}
    mocap_id = {n: model.body(n).mocapid[0]
                for n in ('desk', 'piece', 'dropbox')}
    holding = {'v': False}          # 물기 상태 추적 — 방출 시 낙하 표현용

    # 책상: 상면을 실측 floor 높이에 (박스 반높이 0.03 만큼 내려 배치)
    desk_c = panel_to_sim((0.15, 0.0, floor), R, t)
    data.mocap_pos[mocap_id['desk']] = desk_c - np.array([0, 0, 0.03])

    def set_piece(xy):
        h = 0.011 if a.piece == 'lying' else 0.035
        data.mocap_pos[mocap_id['piece']] = panel_to_sim(
            (xy[0], xy[1], floor + h), R, t)
        data.mocap_quat[mocap_id['piece']] = (
            LYING_QUAT if a.piece == 'lying' else (1, 0, 0, 0))

    def set_pose_deg(deg):
        q = arm_lib.servo_to_rad({f'{j}.pos': deg[j] for j in JN}, MP)
        for j, v in zip(JN, q):
            if j == 'wrist_roll':
                v = (v + ROLL_OFFSET_RAD + math.pi) % (2 * math.pi) - math.pi
            data.qpos[jadr[j]] = v
        if 'gripper' in deg:
            data.qpos[jadr['gripper']] = math.radians(
                max(-10.0, min(100.0, deg['gripper'])))
        mujoco.mj_forward(model, data)
        # 물체를 문 상태(닫힘)면 체스말을 그리퍼 끝에 붙여 같이 움직인다 —
        # 실팔이 집으면 시뮬도 같이 집는다 (2026-08-20 사용자 지시).
        now_holding = deg.get('gripper', 100) < GRIP_HOLD_DEG
        if holding['v'] and not now_holding and not a.piece_at:
            # 방출 순간 — 물체를 그 자리 수직 아래(바닥/박스 바닥)로 떨어뜨린다.
            # 뎁스캠이 박스 안을 못 보므로 blob 갱신은 기대할 수 없다.
            p = data.mocap_pos[mocap_id['piece']].copy()
            drop_z = panel_to_sim((0.0, 0.0, floor + 0.011), R, t)[2]
            data.mocap_pos[mocap_id['piece']] = (p[0], p[1], drop_z)
        holding['v'] = now_holding
        if now_holding and not a.piece_at:
            g = data.site('graspframe').xpos
            pan_w = R @ np.array(arm_lib.PAN0) + t     # 팬 축의 월드 좌표
            dx, dy = g[0] - pan_w[0], g[1] - pan_w[1]
            # 물체는 팬 축 기준 방사 방향으로 물려 바깥을 향한다(손목캠 실측).
            # 조립 각도는 근사 표시용.
            n = math.hypot(dx, dy) or 1.0
            ux, uy = dx / n, dy / n
            half = math.sqrt(0.5)
            # 원기둥 z축을 수평 (ux,uy,0) 으로 돌리는 quat: z→dir 회전축 = z×dir
            data.mocap_pos[mocap_id['piece']] = (
                g[0] + ux * 0.02, g[1] + uy * 0.02, g[2])
            data.mocap_quat[mocap_id['piece']] = (
                half, -uy * half, ux * half, 0.0)
            mujoco.mj_forward(model, data)

    he_R = he_t = None
    hep = D.parent / 'handeye.json'
    if hep.exists():
        he = json.loads(hep.read_text())
        he_R, he_t = np.array(he['R']), np.array(he['t'])

    if a.piece_at:
        set_piece([float(v) for v in a.piece_at.split(',')])

    if a.deg:                                     # 정지 자세 (서버 불필요)
        deg = {k: float(v) for k, v in
               (kv.split('=') for kv in a.deg.split(','))}
        missing = [j for j in JN if j not in deg]
        assert not missing, f'--deg 에 관절 누락: {missing}'
        set_pose_deg(deg)
    else:
        st = get('/state')                        # 시작 자세 = 현재 실팔
        set_pose_deg(st['pos'])

    if a.snapshot:
        import os
        os.environ.setdefault('MUJOCO_GL', 'egl')
        r = mujoco.Renderer(model, height=720, width=1280)
        if a.cam:
            r.update_scene(data, camera=a.cam)
        else:
            cam = mujoco.MjvCamera()
            cam.lookat[:] = panel_to_sim((0.15, 0.0, 0.0), R, t)
            cam.distance, cam.azimuth, cam.elevation = 0.9, 150, -25
            r.update_scene(data, camera=cam)
        px = r.render()
        import PIL.Image
        PIL.Image.fromarray(px).save(a.snapshot)
        print(f'스냅샷 저장: {a.snapshot}')
        return

    if a.record:
        import os
        os.environ.setdefault('MUJOCO_GL', 'egl')
        import PIL.Image
        r = mujoco.Renderer(model, height=720, width=1280)
        cam = None
        if not a.cam:
            cam = mujoco.MjvCamera()
            cam.lookat[:] = panel_to_sim((0.15, 0.0, 0.0), R, t)
            cam.distance, cam.azimuth, cam.elevation = 0.9, 150, -25
        outd = pathlib.Path(a.record)
        outd.mkdir(parents=True, exist_ok=True)
        n = 0
        t_end = time.monotonic() + a.seconds
        print(f'녹화 시작 — {a.fps:.0f}fps · {a.seconds:.0f}s · {outd}')
        while time.monotonic() < t_end:
            t0 = time.monotonic()
            try:
                st = get('/state', timeout=1.0)
                set_pose_deg(st['pos'])
                if (he_R is not None and not a.piece_at
                        and st['pos'].get('gripper', 0) >= 25
                        and n % int(a.fps) == 0):   # 열림 상태면 물체는 뎁스캠 위치
                    xy = read_blob_xy(he_R, he_t, floor,
                                      0.011 if a.piece == 'lying' else 0.035)
                    if xy:
                        set_piece(xy)
            except Exception:
                pass
            if a.cam:
                r.update_scene(data, camera=a.cam)
            else:
                r.update_scene(data, camera=cam)
            PIL.Image.fromarray(r.render()).save(outd / f'f{n:05d}.png')
            n += 1
            time.sleep(max(0.0, 1.0 / a.fps - (time.monotonic() - t0)))
        print(f'녹화 끝 — {n} 프레임 → {outd}')
        return

    import mujoco.viewer
    print('미러 시작 — 창을 닫으면 종료 (팔 명령 없음, 읽기 전용)')
    with mujoco.viewer.launch_passive(model, data) as v:
        n = 0
        while v.is_running():
            t0 = time.monotonic()
            if not a.deg:
                try:
                    st = get('/state', timeout=1.0)
                    set_pose_deg(st['pos'])
                except Exception:
                    pass                          # 서버 순단 — 마지막 자세 유지
                n += 1
                if (he_R is not None and not a.piece_at
                        and n % int(a.hz) == 0):  # 물체는 1Hz 갱신
                    try:
                        xy = read_blob_xy(he_R, he_t, floor,
                                          0.011 if a.piece == 'lying' else 0.035)
                        if xy:
                            set_piece(xy)
                    except Exception:
                        pass
            v.sync()
            time.sleep(max(0.0, 1.0 / a.hz - (time.monotonic() - t0)))


if __name__ == '__main__':
    main()
