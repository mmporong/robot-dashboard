"""추적 JSONL·프레임 JSON → 시행당 MCAP 한 개. **레거시 기록 변환기.**

지금까지 이 프로젝트는 자체 JSONL로 기록했다. 자체 포맷은 이 기록을 이 프로젝트
밖에서 못 쓰게 만든다 — Foxglove로도, `ros2 bag play`로도, 남의 도구로도 안 열린다.
그래서 **읽는 쪽을 MCAP 하나로 고정**하고(`read_mcap.py`), 기록은 여기서 표준 포맷으로
옮긴다. 시행 스크립트가 `ros2 bag record`로 직접 MCAP을 쓰게 되면 이 파일은 쓸 일이
없어진다. 그때까지는 시행이 돌 때마다 여기를 거친다.

**세대(epoch)를 파일에 적는다.** 무대 배치·로봇 설정이 바뀌면 그 전후 시행은 같은
잣대로 비교할 수 없다. 세대를 읽는 쪽에서 id로 짐작하게 두면 `ros2 bag record`가 만든
파일에는 아예 쓸 수가 없다 — 기록의 성질이므로 기록에 적는다.

**어느 토픽에 무엇을 싣나** — 전부 실재하는 ROS 2 타입이다. 이 프로젝트 전용 타입을
새로 만들지 않는다(자체 포맷을 안 만들기로 한 것과 같은 이유).

| 토픽 | 타입 | 실린 것 |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | 시뮬 시계 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 팔 5축 + `arm_gripper`. 그리퍼 부하는 `effort` |
| `/tf` | `tf2_msgs/msg/TFMessage` | `base_footprint`→`arm_gripper_frame_link` (죠 위치) |
| `/odom` | `nav_msgs/msg/Odometry` | 차체 주행 추정 [x, y, yaw] |
| `/gt/cube` `/gt/robot` `/gt/trash` | `geometry_msgs/msg/PoseStamped` | Gazebo 실좌표 (`world`) |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 단계·판정·코드주장과 나머지 필드 전부 |
| `/rgbd_camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | 전방캠 JPEG |
| `/wrist_camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 손목캠 JPEG |

**판정은 싣지 않는다.** 통 안에 들어갔는지는 `/gt/*`에서 읽는 쪽이 다시 계산한다.
판정 결과를 파일에 박아 두면 그 순간 이 대시보드는 "코드가 뭐라고 했는지"를 되풀이하는
물건이 된다 — 이 프로젝트가 밝힌 것이 바로 그 주장이 틀렸다는 사실이다.

**시계**: `log_time`은 시뮬 시각(0 기준)이다. 벽시계는 기록에 없다. RTF≥0.9를 강제한
기록이라 시뮬 초 ≈ 실제 초지만, 1970년으로 보이는 것은 그 때문이다.

**기록 순서**: 옛 JSONL은 시간순이 아니었다(복구 사이클이 끼면 뒤섞인다 — 주행 #13이
212.1 → 168.4 → 217.8). bag은 정의상 시간순이라 여기서 세워 넣는다. 읽는 쪽에서
정렬하던 군더더기가 이 마이그레이션으로 없어진다.

사용: python3 to_mcap.py [출력디렉터리]
"""
import base64
import json
import math
import pathlib
import sys
import types

import rosmsg
from mcap_ros2.writer import Writer

HOME = pathlib.Path.home()
LOGS = HOME / 'capstone_tools/logs'
OUT = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else HOME / 'capstone_tools/mcap'

# 어느 기록이 어느 세대인지. 세대가 갈리는 이유를 여기 적어 둔다 — 나중에 추세를
# 볼 때 "왜 여기서 경계를 그었나"를 코드 밖에서 찾아 헤매지 않도록.
JOBS = [
    # 팔 단독 1세대 — 월드에 큐브가 **하나뿐이던** 무대. 방해 큐브가 없어 차체가
    # 올라탈 일이 없었고, 그래서 3/3이 나왔다. 원본은 이후 새 시행과 섞여
    # `armonly_trace.mixed_backup.jsonl`이 됐으므로 이관 전 백업에서 읽는다.
    {'src': LOGS / 'mcap_backup_20260813/armonly_trace.jsonl', 'kind': '팔 단독',
     'prefix': 'A', 'epoch': 'arm-v1', 'trash': 'gz',
     'frames': LOGS / 'mcap_backup_20260813/armonly_frames.json'},
    # 팔 단독 2세대 — 2026-08-13 무대 개편. 방해 큐브를 벽 속(3.0,2.4)에 버리던 것을
    # 고치고, 세 큐브를 **차체를 안 움직이고 팔이 닿는 자리**에 모았다. 파지 자세의
    # 도달 반경이 0.35~0.37로 좁아 자리는 `K.grasp_q`가 푸는지로 골랐다.
    {'src': LOGS / 'armonly_trace.jsonl', 'kind': '팔 단독',
     'prefix': 'B', 'epoch': 'arm-v2', 'trash': 'gz'},
    # 주행 — 이동·접근까지 포함. 통 인식에 막혀 실좌표로 통에 들어간 것이 하나도 없다.
    {'src': LOGS / 'trace.jsonl', 'kind': '주행',
     'prefix': 'drive_', 'epoch': 'drive-v1', 'trash': 'default', 'split': '시작'},
]

ARM_JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
              'arm_wrist_flex', 'arm_wrist_roll']
GRIPPER = 'arm_gripper'
# 주행 쪽 기록엔 통 좌표가 없다. 무대 설정값을 쓰되 출처를 메타데이터에 남긴다 —
# 실측이 아닌 값이 실측인 척 섞이면 판정 근거가 무너진다.
TRASH_DEFAULT = (0.15, 0.62, 0.0)

TYPES = {
    '/clock': 'rosgraph_msgs/msg/Clock',
    '/joint_states': 'sensor_msgs/msg/JointState',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/odom': 'nav_msgs/msg/Odometry',
    '/gt/cube': 'geometry_msgs/msg/PoseStamped',
    '/gt/robot': 'geometry_msgs/msg/PoseStamped',
    '/gt/trash': 'geometry_msgs/msg/PoseStamped',
    '/diagnostics': 'diagnostic_msgs/msg/DiagnosticArray',
    '/rgbd_camera/image/compressed': 'sensor_msgs/msg/CompressedImage',
    '/wrist_camera/image_raw/compressed': 'sensor_msgs/msg/CompressedImage',
}
# 전용 토픽이 이미 실어 나르는 필드. 나머지는 전부 진단으로 흘려 아무것도 안 잃는다.
CARRIED = {'t', 'stage', 'joints', 'grip_ang', 'grip_load', 'odom', 'jaw',
           'cube_gz', 'robot_gz', 'trash_gz'}


def msg(v):
    """메시지 트리를 dict 대신 네임스페이스로 세운다.

    인코더는 필드를 `hasattr`로 먼저 찾는다. `dict`에는 `values`라는 메서드가 이미
    있어서 `DiagnosticStatus.values`를 dict로 넘기면 그 메서드가 필드 값으로 잡힌다
    ("Field values is not an array"). 이름이 겹칠 여지를 아예 없앤다.
    """
    if isinstance(v, dict):
        return types.SimpleNamespace(**{k: msg(x) for k, x in v.items()})
    if isinstance(v, list):
        return [msg(x) for x in v]
    return v


def ns(t):
    return int(round(t * 1e9))


def stamp(t):
    sec, rest = divmod(max(0.0, t), 1.0)
    return {'sec': int(sec), 'nanosec': min(999_999_999, int(round(rest * 1e9)))}


def yaw_quat(yaw):
    return {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}


def pose_stamped(t, xyz, frame='world'):
    return {
        'header': {'stamp': stamp(t), 'frame_id': frame},
        'pose': {'position': {'x': xyz[0], 'y': xyz[1], 'z': xyz[2]},
                 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
    }


def diagnostics(t, row):
    """단계 표시와 남은 필드 전부. 값은 문자열로 — KeyValue가 그런 타입이다."""
    verdict, claim = row.get('판정'), row.get('코드주장')
    level = 2 if claim == 'PICK_FAIL' else (1 if verdict in ('EMPTY', 'DROPPED') else 0)
    values = [{'key': k, 'value': json.dumps(v, ensure_ascii=False) if isinstance(v, (list, dict))
               else str(v)}
              for k, v in sorted(row.items()) if k not in CARRIED and v is not None]
    return {
        'header': {'stamp': stamp(t), 'frame_id': ''},
        'status': [{
            'level': level,
            'name': 'capstone_pick',
            'message': row.get('stage') or '',
            # 코드가 스스로 무엇이라 주장했는지는 남기되, 판정 근거로는 쓰지 않는다
            'hardware_id': claim or verdict or '',
            'values': values,
        }],
    }


def write_run(path, rows, frames, meta):
    """시행 하나를 MCAP 한 개로. 프레임은 자기 시각에 실린다."""
    rows = sorted(rows, key=lambda r: r['t'])
    # 시행의 0초는 단계와 프레임을 통틀어 가장 이른 시각이다. 둘을 따로 0으로 맞추면
    # (옛 파이프라인이 그랬다) 녹화가 첫 단계보다 먼저 시작된 시행에서 영상과 관절이
    # 어긋난 채 겹쳐진다. bag에 넣는 이상 시계는 하나여야 한다.
    t0 = min([r['t'] for r in rows] + [f['t'] for fs in frames.values() for f in fs])
    w = Writer(path.open('wb'))
    sch, counts = {}, dict.fromkeys(TYPES, 0)

    def put(topic, body, t):
        # 스키마 본문은 파일 안에 실린다(그래서 ROS 없이 열린다). 다만 한 개가 2~4KB라
        # 안 쓰는 토픽까지 미리 등록하면 짧은 시행에서 본문보다 스키마가 커진다.
        if topic not in sch:
            sch[topic] = w.register_msgdef(TYPES[topic], rosmsg.msgdef(TYPES[topic]))
        w.write_message(topic, sch[topic], msg(body), log_time=ns(t), publish_time=ns(t))
        counts[topic] += 1

    for r in rows:
        t = round(r['t'] - t0, 3)
        put('/clock', {'clock': stamp(t)}, t)
        jp = r.get('joints') or {}
        put('/joint_states', {
            'header': {'stamp': stamp(t), 'frame_id': ''},
            'name': ARM_JOINTS + [GRIPPER],
            'position': [float(jp.get(k, 0.0)) for k in ARM_JOINTS] + [float(r.get('grip_ang') or 0.0)],
            'velocity': [],
            # 그리퍼 부하만 기록돼 있다. 없는 값을 0으로 채우면 "힘이 0이었다"는 거짓이 된다
            'effort': [float('nan')] * len(ARM_JOINTS) + [float(r.get('grip_load') or 0.0)],
        }, t)
        if r.get('jaw'):
            # 기록에 회전은 없다(`lookup_transform`의 translation만 남겼다). 단위 사원수는
            # 자리를 채우는 값일 뿐이라 읽는 쪽도 위치만 쓴다.
            put('/tf', {'transforms': [{
                'header': {'stamp': stamp(t), 'frame_id': 'base_footprint'},
                'child_frame_id': 'arm_gripper_frame_link',
                'transform': {
                    'translation': dict(zip('xyz', map(float, r['jaw']))),
                    'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
            }]}, t)
        if r.get('odom'):
            x, y, yaw = r['odom']
            put('/odom', {
                'header': {'stamp': stamp(t), 'frame_id': 'odom'},
                'child_frame_id': 'base_footprint',
                'pose': {'pose': {'position': {'x': float(x), 'y': float(y), 'z': 0.0},
                                  'orientation': yaw_quat(float(yaw))},
                         'covariance': [0.0] * 36},
                'twist': {'twist': {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                          'covariance': [0.0] * 36},
            }, t)
        for key, topic in (('cube_gz', '/gt/cube'), ('robot_gz', '/gt/robot'),
                           ('trash_gz', '/gt/trash')):
            if r.get(key):
                put(topic, pose_stamped(t, [float(v) for v in r[key]]), t)
        put('/diagnostics', diagnostics(t, r), t)

    for cam, topic in (('front', '/rgbd_camera/image/compressed'),
                       ('wrist', '/wrist_camera/image_raw/compressed')):
        for f in sorted(frames.get(cam, []), key=lambda f: f['t']):
            t = round(f['t'] - t0, 3)
            put(topic, {
                'header': {'stamp': stamp(t), 'frame_id': f'{cam}_camera'},
                'format': 'jpeg',
                'data': base64.b64decode(f['b']),
            }, t)

    w._writer.add_metadata('run', {k: str(v) for k, v in meta.items()})
    w.finish()
    return counts


def rows_of(path):
    if not path.exists():
        return []
    return [json.loads(ln) for ln in path.read_text(encoding='utf-8').splitlines() if ln.strip()]


def split_runs(job):
    """한 추적 파일을 시행 단위로 가른다.

    `trial` 필드가 있으면 그것으로, 없으면 `split`에 적힌 단계 이름이 나올 때마다 끊는다.
    """
    rows = rows_of(job['src'])
    # 파일명만 적으면 안 된다 — 1세대와 2세대의 원본이 둘 다 `armonly_trace.jsonl`이고
    # 내용이 다르다. 어느 쪽에서 나온 기록인지가 세대 구분의 근거라 경로째 적는다.
    base = {'kind': job['kind'], 'epoch': job['epoch'], 'trash_source': job['trash'],
            'source': str(job['src']).replace(str(HOME), '~')}
    if job.get('split'):
        cur, rid = [], 0
        for r in rows + [{'stage': job['split'], '__end__': True}]:
            if r.get('stage') == job['split'] and cur:
                yield f'{job["prefix"]}{rid}', cur, dict(base, trial=rid)
                rid += 1
                cur = []
            if not r.get('__end__'):
                cur.append(r)
        return
    by = {}
    for r in rows:
        by.setdefault(r.get('trial', 0), []).append(r)
    for trial, rs in sorted(by.items()):
        yield f'{job["prefix"]}{trial}', rs, dict(base, trial=trial)


def frames_by_run(job):
    """녹화가 있으면 시행별로 가른다. 주행 쪽은 그때 녹화를 하지 않아 비어 있다."""
    src = job.get('frames')
    if not src or not src.exists():
        return {}
    out = {}
    for cam, fs in json.loads(src.read_text(encoding='utf-8')).items():
        for f in fs:
            out.setdefault(f'{job["prefix"]}{f["trial"]}', {}).setdefault(cam, []).append(f)
    return out


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    jobs = [(rid, rows, meta, frames)
            for job in JOBS
            for frames in [frames_by_run(job)]
            for rid, rows, meta in split_runs(job)]

    total = 0
    for rid, rows, meta, frames_all in jobs:
        by_trial = frames_all
        rows = [r for r in rows if r.get('t') is not None]
        if len(rows) < 3:
            print(f'  {rid}: 기록 {len(rows)}행 — 건너뜀')
            continue
        if meta['trash_source'] == 'default':
            # 주행 기록엔 통 좌표가 없다. 없으면 판정 자체가 불가능하므로 무대 설정값을
            # 넣되, 출처를 메타데이터에 남겨 실측과 구분되게 한다.
            for r in rows:
                r.setdefault('trash_gz', list(TRASH_DEFAULT))
        path = OUT / f'{rid}.mcap'
        meta = dict(meta, id=rid, rows=len(rows), clock='sim')
        counts = write_run(path, rows, by_trial.get(rid, {}), meta)
        size = path.stat().st_size
        total += size
        cams = counts['/rgbd_camera/image/compressed'] + counts['/wrist_camera/image_raw/compressed']
        print(f'  {path.name:14s} {size // 1024:6d}KB · {counts["/joint_states"]:4d}단계'
              f' · 카메라 {cams:4d}장 · gt {counts["/gt/cube"]}')
    n = len(list(OUT.glob('*.mcap')))
    print(f'{OUT} — MCAP {n}개 · 합계 {total / 1e6:.1f}MB')


if __name__ == '__main__':
    main()
