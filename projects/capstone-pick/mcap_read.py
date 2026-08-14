"""MCAP → `runs.json` · `run_frames.json` — **캡스톤 pick 전용 리더.**

중립 부분(파일 열기·TF 합성·시각 조회·용량 예산)은 `core/mcap_io.py`에 있다.
여기 남은 것은 이 과제에만 뜻이 있는 것들이다 — `/joint_states`에서 팔 관절을 이름으로
찾고, `/gt/cube`·`/gt/trash`로 통 안인지 다시 세고, `/diagnostics`의 단계 표시를 읽는다.

**판정은 파일에 들어 있지 않다.** 실좌표만 싣고 통 안인지는 여기서 다시 센다. 결론을
파일에 박으면 이 화면은 코드가 뭐라고 했는지를 되풀이하는 물건이 된다 — 그 둘이
어긋난다는 것이 이 프로젝트가 찾아낸 사실이다.

**받는 방식**은 Foxglove Agent와 같은 계약이다 — *완성된 파일이 디렉터리에 떨어져
있으면 집는다.* 스트리밍이 아니고, 반쯤 쓰인 파일을 집지 않도록 `.mcap`으로 이름이
바뀐 것만 본다.

사용: `python3 dash.py capstone-pick read`
"""
import json
import math
import os
import pathlib

import mcap_io as IO
import project as P
import video as V

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
    out, tree = [], IO.tf_tree(series)
    for t in step_times(series):
        js = IO.at(series.get('/joint_states', []), t, MAX_AGE)
        if js is None:
            continue
        idx = {n: i for i, n in enumerate(js.name)}
        pos, eff = list(js.position), list(js.effort)
        jaw = IO.tf_pos(tree, JAW_FRAME, BASE_FRAME, t, MAX_AGE)
        od = IO.at(series.get('/odom', []), t, MAX_AGE)
        cube = IO.at(series.get('/gt/cube', []), t, MAX_AGE)
        robot = IO.at(series.get('/gt/robot', []), t, MAX_AGE)
        diag = IO.at(series.get('/diagnostics', []), t, MAX_AGE)
        kv, stage = {}, None
        if diag and diag.status:
            st = diag.status[0]
            stage = st.message or None
            kv = {p.key: p.value for p in st.values}
        gi = idx.get(GRIPPER)
        oq = od.pose.pose.orientation if od is not None else None
        out.append({
            't': round(t, 1),
            'stage': stage,
            'q': [round(pos[idx[n]], 4) if n in idx and idx[n] < len(pos) else 0.0
                  for n in ARM_ORDER],
            'ga': round(pos[gi], 3) if gi is not None and gi < len(pos) else None,
            'gl': round(eff[gi], 3) if gi is not None and gi < len(eff) else None,
            'odom': None if od is None else [round(od.pose.pose.position.x, 3),
                                             round(od.pose.pose.position.y, 3),
                                             round(IO.yaw_of(oq.x, oq.y, oq.z, oq.w), 3)],
            'jaw': jaw,
            'cube': None if cube is None else IO.xyz(cube.pose.position),
            'robot': None if robot is None else IO.xyz(robot.pose.position),
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
        # 물체 크기는 **바닥에 놓였을 때의** 중심 높이에서 나온다 — 무대가 3cm/4cm를
        # 오갔기 때문에 상수로 박으면 게이지의 기준선이 틀린다.
        #
        # 첫 표본을 쓰면 안 된다. 기록이 시작될 때 물체가 이미 들려 있을 수 있다
        # (bagA3이 그랬다: 첫 관측 0.027 → 54mm로 잡혀 실제 30mm의 두 배가 됐다).
        # 물체는 바닥보다 낮아질 수 없으므로 **관측된 최소 높이**가 곧 반지름이다.
        # 물리 떨림으로 0.2mm쯤 파고드는 표본이 있지만 소수 세 자리에서 반올림되며 사라진다.
        'cube_size': round(min(c[2] for c in cubes) * 2, 3) if cubes else None,
    }


def run_of(path):
    """MCAP 하나 → (메타, 판정된 시행). 판정 불가면 시행이 None이다.

    누적·회귀 쪽(`core/regress.py`)이 부르는 **유일한 입구**다. 그쪽이 `load`·`steps_of`·
    `judge`를 직접 부르면 프로젝트마다 다른 그 셋에 묶여 중립일 수가 없다.
    """
    # **카메라는 풀지 않는다.** 누적·회귀는 좌표만 보는데, 영상까지 풀면 시행 하나가
    # 수십 MB짜리 base64 덩어리가 된다(재녹화본이 시행당 700장·640×480이다).
    meta, series, _ = IO.load(path, {}, JPEG)
    return meta, judge(path.stem, meta, series, steps_of(series))



def probe(path):
    """인박스에 떨어진 파일 하나가 쓸 만한지 본다.

    감시 디렉터리로 받는 이상 남이 만든 파일이 들어온다. 못 쓰면 **왜 못 쓰는지**를
    말해야 한다 — "판정 불가"만 찍고 마는 화면은 파일이 잘못된 건지 시행이 실패한
    건지 구분해 주지 않는다.
    """
    meta, series, frames = IO.load(path, CAMS, JPEG)
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
        meta, series, frames = IO.load(path, CAMS, JPEG)
        rec = judge(rid, meta, series, steps_of(series))
        if rec is None:
            skipped.append(rid)
            continue
        IO.rebase(rec, frames)
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

    if getattr(P, 'VIDEO', False):
        # 비디오는 고정 주기라야 담긴다. 격자에 맞춰 고르고(영차 유지) 한 번에 굽는다.
        span = {rec['id']: rec['dur'] for rec in runs}
        for rid, cams in by_run.items():
            for cam in cams:
                cams[cam] = V.resample(cams[cam], P.VIDEO_FPS, span.get(rid, 0))
        budget = getattr(P, 'VIDEO_BUDGET_MB', BUDGET_MB)
        by_run, total = V.build(by_run, P.VIDEO_FPS, budget, crf=P.VIDEO_CRF)
        print(f'  비디오 {total/1e6:.1f}MB (예산 {budget}MB)')
    else:
        IO.fit_budget(by_run, BUDGET_MB, {'wrist': 1.0, 'front': 0.45})

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
            fs = by_run[rid].get(cam)
            if not fs:
                continue
            if isinstance(fs, dict):        # 비디오
                print(f'  {rid} {cam:5s} {fs["n"]:4d}프레임 · {fs["w"]}x{fs["h"]}'
                      f' · {fs["fps"]}fps · {len(fs["src"]) * 3 // 4 // 1024}KB')
                continue
            gap = (fs[-1]['t'] - fs[0]['t']) / max(1, len(fs) - 1)
            kb = sum(len(f['b']) for f in fs) // 1024
            print(f'  {rid} {cam:5s} {len(fs):4d}장 · {fs[0]["t"]:.1f}~{fs[-1]["t"]:.1f}s '
                  f'· 평균 간격 {gap:.2f}s · {kb}KB')


if __name__ == '__main__':
    main()
