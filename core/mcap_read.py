"""MCAP → `runs.json` · `run_frames.json` — 대시보드로 들어오는 **유일한 입구**.

전에는 이 프로젝트 전용 JSONL을 읽었다. 자체 포맷은 기록을 이 프로젝트 안에 가둔다.
지금은 입구가 MCAP 하나다. 그래서 여기로 들어오는 것은 세 가지가 모두 같은 자격이다.

  · `to_mcap.py`가 옛 JSONL을 옮겨 놓은 파일 (일회성 마이그레이션)
  · `ros2 bag record`가 직접 쓴 파일
  · 남이 준 MCAP — Foxglove·rosbag2·MCAP SDK 어느 쪽이 만든 것이든

**ROS는 필요 없다.** MCAP은 메시지 정의를 파일 안에 넣고 다니므로 읽는 쪽은 스키마를
파일에서 꺼내 푼다. 이 스크립트는 `/opt/ros`가 없는 기계에서도 돈다.

**받는 방식**은 Foxglove Agent가 쓰는 계약과 같게 잡았다 — *완성된 파일이 디렉터리에
떨어져 있으면 집는다.* 스트리밍이 아니다. 반쯤 쓰인 파일을 집지 않도록 `.mcap`으로
이름이 바뀐 것만 본다.

**판정은 여기서 한다.** 파일에는 실좌표(`/gt/*`)만 들어 있고 "성공"이라는 결론은 없다.
개구부 반폭 68mm 안이고 벽 상단 0.090m보다 낮아야 통 안이다. 코드가 뭐라고 주장했는지
(`/diagnostics`의 `코드주장`)는 나란히 보여 주기만 한다 — 그 둘이 어긋난다는 것이
이 프로젝트가 찾아낸 사실이라, 주장을 판정으로 승격시키면 화면이 할 말이 없어진다.

사용: python3 read_mcap.py [입력디렉터리] [프레임예산MB]
      python3 read_mcap.py --probe <파일.mcap>     # 이 파일에서 무엇이 읽히나
"""
import base64
import bisect
import io
import json
import math
import os
import pathlib
import sys

from mcap.reader import make_reader
from mcap_ros2.reader import DecoderFactory, read_ros2_messages

import project as P

HERE = pathlib.Path.cwd()          # 생성물은 프로젝트 폴더에 떨어진다
# 기본값만 여기서 정한다. **import 시점에 `sys.argv`를 읽지 않는다** — 이 파일을 모듈로
# 가져다 쓰는 쪽(`trend.py`·`to_lerobot.py`)은 자기 인자를 쓰는데, 여기서 argv를 해석하면
# 남의 인자에 걸려 import가 그 자리에서 죽는다(실제로 겪었다).
INBOX = pathlib.Path.home() / 'capstone_tools/mcap'
BUDGET_MB = P.FRAME_BUDGET_MB

# 프로젝트 고유 값은 전부 `project.py`에서 온다. 이 파일에 로봇 이름이나 판정 규칙을
# 다시 적지 않는다 — 적는 순간 다른 프로젝트로 못 옮긴다.
OPEN_HALF, WALL_TOP = P.OPEN_HALF, P.WALL_TOP
ARM_ORDER, GRIPPER = P.ARM_ORDER, P.GRIPPER
JAW_FRAME, BASE_FRAME = P.JAW_FRAME, P.BASE_FRAME
CAMS, SHOW, LABEL, JPEG = P.CAMS, P.SHOW, P.LABEL, P.JPEG
MAX_AGE = 1.0
# 단계 표시가 없는 bag(`ros2 bag record`가 그냥 받아 적은 것)에서 시간축을 세울 간격.
# 원시 표본을 그대로 단계로 쓰면 50Hz 기록이 단계 수천 개가 된다.
STEP_HZ = 1.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def xyz(p):
    return [round(p.x, 4), round(p.y, 4), round(p.z, 4)]


def at(series, t, eps=1e-6):
    """t 시점의 값. 같은 시각에 실린 것까지 포함해 가장 최근 것을 고른다.

    `MAX_AGE`보다 오래된 표본은 버린다. 끌고 오면 화면이 "그때 큐브가 거기 있었다"고
    말하게 되는데, 그건 기록이 아니라 추정이다 — 실좌표로 판정한다는 전제가 무너진다.
    """
    i = bisect.bisect_right(series, t + eps, key=lambda p: p[0])
    if not i:
        return None
    ts, v = series[i - 1]
    return v if t - ts <= MAX_AGE else None


def to_jpeg(m, topic):
    """카메라 메시지를 JPEG 바이트로. 압축 토픽이면 그대로, 원본이면 여기서 줄여 굽는다."""
    if topic.endswith('/compressed'):
        return bytes(m.data)
    from PIL import Image                      # 원본 이미지가 실린 bag에서만 필요하다
    import numpy as np
    ch = {'rgb8': 3, 'bgr8': 3, 'mono8': 1, 'rgba8': 4, 'bgra8': 4}.get(m.encoding)
    if ch is None:
        return None
    a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, ch)
    if m.encoding.startswith('bgr'):
        a = a[:, :, ::-1] if ch == 3 else a[:, :, [2, 1, 0, 3]]
    img = Image.fromarray(a.squeeze())
    img = img.resize((max(1, int(m.width * JPEG['scale'])), max(1, int(m.height * JPEG['scale']))))
    buf = io.BytesIO()
    img.convert('RGB').save(buf, 'JPEG', quality=JPEG['quality'])
    return buf.getvalue()


def load(path):
    """MCAP 한 개 → (메타데이터, 토픽별 시계열, 카메라 프레임)."""
    series, frames, meta = {}, {'front': [], 'wrist': []}, {}
    with path.open('rb') as f:
        # 메타데이터와 메시지를 한 번에 훑으려면 리더를 직접 세워야 한다. 이때
        # ROS 2 해독기를 함께 물려야 스키마가 파일 안에 있어도 메시지가 풀린다.
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for md in reader.iter_metadata():
            if md.name == 'run':
                meta = dict(md.metadata)
        for m in read_ros2_messages(reader):
            topic, t = m.channel.topic, m.log_time_ns / 1e9
            if topic in CAMS:
                b = to_jpeg(m.ros_msg, topic)
                if b:
                    frames[CAMS[topic]].append({'t': round(t, 2), 'b': base64.b64encode(b).decode()})
            else:
                series.setdefault(topic, []).append((t, m.ros_msg))
    return meta, series, frames


def tf_tree(series):
    """`/tf`·`/tf_static`을 자식 프레임으로 색인한다. 고정 관절은 static 쪽에만 있다."""
    dyn, static = {}, {}
    for t, m in series.get('/tf', []):
        for tr in m.transforms:
            dyn.setdefault(tr.child_frame_id, []).append((t, tr))
    for _, m in series.get('/tf_static', []):
        for tr in m.transforms:
            static[tr.child_frame_id] = tr
    return dyn, static


def qrot(q, v):
    """사원수 q로 벡터 v를 돌린다 (v + 2·qv × (qv × v + w·v))."""
    u = (q.x, q.y, q.z)
    c1 = (u[1] * v[2] - u[2] * v[1] + q.w * v[0],
          u[2] * v[0] - u[0] * v[2] + q.w * v[1],
          u[0] * v[1] - u[1] * v[0] + q.w * v[2])
    c2 = (u[1] * c1[2] - u[2] * c1[1],
          u[2] * c1[0] - u[0] * c1[2],
          u[0] * c1[1] - u[1] * c1[0])
    return (v[0] + 2 * c2[0], v[1] + 2 * c2[1], v[2] + 2 * c2[2])


def tf_pos(tree, target, root, t):
    """t 시점에서 `root` 기준 `target` 원점의 위치.

    `/tf`는 **부모→자식 한 칸씩만** 싣는다 — `base_footprint`에서 죠까지의 변환이
    통째로 실려 오는 일은 없다. 그래서 자식에서 부모로 거슬러 올라가며 합성한다
    (`tf2`의 `lookup_transform`이 하는 일). 변환기가 만든 파일은 한 칸이면 끝나고,
    `ros2 bag record`가 받아 적은 파일은 관절 수만큼 거슬러 올라간다.
    """
    dyn, static = tree
    p, frame = (0.0, 0.0, 0.0), target
    for _ in range(64):                    # 트리가 고리를 이루면 여기서 멈춘다
        if frame == root:
            return [round(v, 4) for v in p]
        tr = at(dyn.get(frame, []), t) or static.get(frame)
        if tr is None:
            return None
        p = qrot(tr.transform.rotation, p)
        d = tr.transform.translation
        p = (p[0] + d.x, p[1] + d.y, p[2] + d.z)
        frame = tr.header.frame_id
    return None


def step_times(series):
    """시간축을 어디에 세울지.

    상태 전이가 기록돼 있으면(`/diagnostics`) 그것이 곧 단계다 — 화면이 보여 주는
    "선회·하강·파지"가 그 전이다. 없으면 일정 간격으로 다시 뜬다.
    """
    diag = series.get('/diagnostics')
    if diag:
        return [t for t, _ in diag]
    out, last = [], None
    for t, _ in series.get('/joint_states', []):
        if last is None or t - last >= 1.0 / STEP_HZ:
            out.append(t)
            last = t
    return out


def steps_of(series):
    out, tree = [], tf_tree(series)
    for t in step_times(series):
        js = at(series.get('/joint_states', []), t)
        if js is None:
            continue
        idx = {n: i for i, n in enumerate(js.name)}
        pos, eff = list(js.position), list(js.effort)
        jaw = tf_pos(tree, JAW_FRAME, BASE_FRAME, t)
        od = at(series.get('/odom', []), t)
        cube = at(series.get('/gt/cube', []), t)
        robot = at(series.get('/gt/robot', []), t)
        diag = at(series.get('/diagnostics', []), t)
        kv, stage = {}, None
        if diag and diag.status:
            st = diag.status[0]
            stage = st.message or None
            kv = {p.key: p.value for p in st.values}
        gi = idx.get(GRIPPER)
        out.append({
            't': round(t, 1),
            'stage': stage,
            'q': [round(pos[idx[n]], 4) if n in idx and idx[n] < len(pos) else 0.0
                  for n in ARM_ORDER],
            'ga': round(pos[gi], 3) if gi is not None and gi < len(pos) else None,
            'gl': round(eff[gi], 3) if gi is not None and gi < len(eff) else None,
            'odom': None if od is None else [round(od.pose.pose.position.x, 3),
                                             round(od.pose.pose.position.y, 3),
                                             round(yaw_of(od.pose.pose.orientation), 3)],
            'jaw': jaw,
            'cube': None if cube is None else xyz(cube.pose.position),
            'robot': None if robot is None else xyz(robot.pose.position),
            'claim': kv.get('코드주장'),
            'verdict': kv.get('판정'),
            'er': float(kv['er_mm']) if 'er_mm' in kv else None,
            'brg': float(kv['brg_deg']) if 'brg_deg' in kv else None,
        })
    return out


def judge(rid, meta, series, steps):
    """실좌표로 다시 판정한다. 파일에 실린 주장은 옆에 두고 보기만 한다."""
    steps = [s for s in steps if s['ga'] is not None and s['jaw']]
    if len(steps) < 3:
        return None
    trash = series.get('/gt/trash') or []
    if not trash:
        return None                 # 기준점이 없으면 거리도 판정도 못 낸다
    tp = trash[-1][1].pose.position
    cubes = [s['cube'] for s in steps if s['cube']]
    last = cubes[-1] if cubes else None
    dist = truth = None
    if last:
        dx, dy = last[0] - tp.x, last[1] - tp.y
        dist = round(math.hypot(dx, dy) * 1000)
        truth = P.judge(last, [tp.x, tp.y, tp.z])
    return {
        'id': LABEL.get(rid, rid),
        'kind': meta.get('kind', '미분류'),
        'dur': steps[-1]['t'],
        'steps': steps,
        'claim': next((s['claim'] for s in reversed(steps) if s['claim']), None),
        'cube': last,
        'truth': truth,
        'dist': dist,
        'trash': [round(tp.x, 4), round(tp.y, 4)],
        # 물체 크기는 바닥에 놓였을 때의 중심 높이에서 나온다 — 무대가 3cm/4cm를
        # 오갔기 때문에 상수로 박으면 게이지의 기준선이 틀린다.
        'cube_size': round(cubes[0][2] * 2, 3) if cubes else None,
    }


def rebase(rec, frames):
    """시행의 0초를 **첫 단계**로 옮긴다. 카메라와 관절을 한 시계 위에 둔 채로.

    bag의 0초는 녹화가 깨어난 순간이라 무대를 세우는 4.5초가 앞에 붙어 있다. 화면의
    0초는 시행이 시작한 순간이어야 하므로 여기서 한 번 민다. **두 계열을 같은 값만큼**
    미는 것이 요점이다 — 옛 파이프라인은 단계와 프레임을 각자 0으로 맞췄고, 그래서
    모든 팔 단독 시행에서 영상이 관절보다 4.5초 앞선 채 겹쳐 재생됐다.
    """
    t0 = rec['steps'][0]['t']
    for s in rec['steps']:
        s['t'] = round(s['t'] - t0, 1)
    rec['dur'] = rec['steps'][-1]['t']
    for cam, fs in frames.items():
        for f in fs:
            f['t'] = round(f['t'] - t0, 2)
        # 시행이 시작하기 전 장면은 화면의 시간축 밖이라 어차피 안 보인다
        frames[cam] = [f for f in fs if f['t'] >= 0]


def thin(seq, keep):
    """시각이 고르게 남도록 솎는다. 앞뒤를 자르면 시작·끝 장면이 통째로 사라진다."""
    if keep >= len(seq):
        return seq
    step = len(seq) / keep
    return [seq[min(len(seq) - 1, int(i * step))] for i in range(keep)]


def fit_budget(by_run):
    """아티팩트 한 장에 다 들어가야 한다. 넘으면 솎되, 솎은 사실을 반드시 찍는다."""
    total = sum(len(f['b']) for r in by_run.values() for cam in r for f in r[cam])
    budget = BUDGET_MB * 1024 * 1024
    if total <= budget:
        return
    # 손목캠을 촘촘히 남긴다. 파지·운반·놓기가 전부 거기서 벌어지고, 전방캠은 파지
    # 거리에서 팔에 가려 몇 초씩 같은 그림이다 — 같은 비율로 깎으면 볼 쪽이 끊긴다.
    w = {'wrist': 1.0, 'front': 0.45}
    weighted = sum(len(f['b']) * w[cam] for r in by_run.values() for cam in r for f in r[cam])
    k = budget / weighted
    print(f'  예산 초과 {total/1e6:.1f}MB > {BUDGET_MB}MB — 솎아낸다 '
          f'(손목 {min(1,k)*100:.0f}% · 전방 {min(1,k*w["front"])*100:.0f}%)')
    for rid, r in by_run.items():
        for cam in r:
            before = len(r[cam])
            r[cam] = thin(r[cam], max(2, int(before * min(1.0, k * w[cam]))))
            if before:
                print(f'    {rid} {cam}: {before} → {len(r[cam])}장')


def probe(path):
    """인박스에 떨어진 파일 하나가 쓸 만한지 본다.

    감시 디렉터리로 받는 이상 남이 만든 파일이 들어온다. 못 쓰면 **왜 못 쓰는지**를
    말해야 한다 — "판정 불가"만 찍고 마는 화면은 파일이 잘못된 건지 시행이 실패한
    건지 구분해 주지 않는다.
    """
    meta, series, frames = load(path)
    print(f'{path.name}  {path.stat().st_size / 1e6:.1f}MB')
    print(f'  메타데이터: {meta or "없음 (ros2 bag record는 안 남긴다)"}')
    for topic in sorted(series):
        ts = [t for t, _ in series[topic]]
        hz = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else 0
        print(f'  {topic:38s} {len(ts):7d}개 · {ts[0]:.1f}~{ts[-1]:.1f}s · {hz:.1f}Hz')
    for cam, fs in frames.items():
        if fs:
            print(f'  카메라 {cam:6s} {len(fs):7d}장 · {fs[0]["t"]:.1f}~{fs[-1]["t"]:.1f}s')
    steps = steps_of(series)
    src = '/diagnostics 상태 전이' if series.get('/diagnostics') else f'{STEP_HZ}Hz 재표집'
    print(f'  단계 {len(steps)}개 ({src})')
    if steps:
        got = {k: sum(1 for s in steps if s[k] is not None)
               for k in ('stage', 'jaw', 'odom', 'cube', 'robot', 'ga')}
        print('  단계가 채워진 정도: ' + ' · '.join(f'{k} {v}/{len(steps)}' for k, v in got.items()))
    rec = judge(path.stem, meta, series, steps)
    if rec:
        mark = '통 안' if rec['truth'] else ('통 밖' if rec['truth'] is False else '판정 불가')
        print(f'  판정 가능 — {mark} · 통까지 {rec["dist"]}mm · {rec["dur"]}s')
    else:
        why = '실좌표(/gt/trash)가 없다' if not series.get('/gt/trash') else '쓸 만한 단계가 3개 미만'
        print(f'  판정 불가 — {why}')


def probe_main(paths):
    if not paths:
        raise SystemExit('볼 파일을 적을 것: dash.py <프로젝트> probe <파일.mcap>')
    for p in paths:
        probe(pathlib.Path(p))


def main(inbox=None, budget=None):
    global INBOX, BUDGET_MB
    if inbox:
        INBOX = pathlib.Path(inbox)
    if budget:
        BUDGET_MB = float(budget)
    files = sorted(INBOX.glob('*.mcap'))
    if not files:
        raise SystemExit(f'MCAP이 없다: {INBOX}')
    print(f'{INBOX} — MCAP {len(files)}개')

    runs, by_run, skipped = [], {}, []
    for path in files:
        rid = path.stem
        if rid not in SHOW:
            continue
        meta, series, frames = load(path)
        rec = judge(rid, meta, series, steps_of(series))
        if rec is None:
            skipped.append(rid)
            continue
        rebase(rec, frames)
        runs.append((rid, rec))
        if any(frames.values()):
            by_run[rec['id']] = frames
    if skipped:
        print(f'  판정 기준이 없어 제외: {", ".join(skipped)}')
    if not runs:
        raise SystemExit('실을 시행이 없다')
    runs.sort(key=lambda pair: SHOW.index(pair[0]))
    runs = [rec for _, rec in runs]

    for r in by_run.values():
        for cam in r:
            r[cam].sort(key=lambda f: f['t'])
    fit_budget(by_run)

    (HERE / 'runs.json').write_text(
        json.dumps(runs, ensure_ascii=False, separators=(',', ':')), encoding='utf-8')
    (HERE / 'run_frames.json').write_text(
        json.dumps(by_run, ensure_ascii=False, separators=(',', ':')), encoding='utf-8')
    print(f'runs.json {os.path.getsize(HERE / "runs.json") // 1024}KB · 시행 {len(runs)}')
    for r in runs:
        mark = '○ 통 안' if r['truth'] else ('✗ 통 밖' if r['truth'] is False else '? 판정불가')
        print(f'  #{str(r["id"]):4s} {r["kind"]:6s} {mark:8s} '
              f'통까지 {str(r["dist"]) + "mm":>7s} · 주장 {str(r["claim"]):12s} · '
              f'물체 {r["cube_size"]}m · {len(r["steps"])}단계 · {r["dur"]}s')
    print(f'run_frames.json {os.path.getsize(HERE / "run_frames.json") // 1024}KB')
    for rid in sorted(by_run):
        for cam in ('front', 'wrist'):
            fs = by_run[rid][cam]
            if not fs:
                continue
            gap = (fs[-1]['t'] - fs[0]['t']) / max(1, len(fs) - 1)
            kb = sum(len(f['b']) for f in fs) // 1024
            print(f'  {rid} {cam:5s} {len(fs):4d}장 · {fs[0]["t"]:.1f}~{fs[-1]["t"]:.1f}s '
                  f'· 평균 간격 {gap:.2f}s · {kb}KB')


if __name__ == '__main__':
    main()
