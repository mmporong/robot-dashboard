"""실기 기록 → runs.json · datasets.json (2026-08-21).

`run_demo.sh` 가 남기는 것은 MCAP 한 덩어리가 아니라 **파일 묶음**이다:

    media/<날짜>/demo_<시각>_wrist.mp4    손목캠
                 demo_<시각>_sim.mp4      MuJoCo 미러
                 demo_<시각>_screen.mp4   패널 화면
                 demo_<시각>_state.csv    관절각·온도(·전압) 시계열
                 demo_<시각>_simrec.log   시뮬 녹화 로그

과거 벤치의 rgb/depth 파일은 legacy 채널로 읽기만 하며 새 차량 기록에는 만들지 않는다.

여기서 하는 일은 그 묶음을 시각으로 엮고, 영상은 예산에 맞게 다시 인코딩해
한 장짜리 HTML 에 실을 수 있는 크기로 만드는 것이다.

**판정하지 않는다.** 실기에는 Gazebo 실좌표 같은 심판이 없고, 로그의 성공 문구는
실물과 어긋난 전례가 있다(2026-08-19). 계측된 사실만 싣고 라벨은 사람이 붙인다.
"""
import base64
import csv
import io
import json
import pathlib
import re
import sys

import project as P

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'core'))
import video as V                                        # noqa: E402

RUN_RE = re.compile(r'^demo_(\d{6})_(\w+?)(_realtime)?\.(mp4|csv|log)$')


def _fmt_time(hhmmss):
    return f'{hhmmss[:2]}:{hhmmss[2:4]}:{hhmmss[4:]}'


def scan_runs():
    """미디어 디렉터리 → 시행 목록 (날짜/시각 오름차순)."""
    runs = {}
    root = pathlib.Path(P.MEDIA_ROOT)
    if not root.exists():
        return []
    for day in sorted(p for p in root.iterdir() if p.is_dir()):
        for f in sorted(day.iterdir()):
            m = RUN_RE.match(f.name)
            if not m:
                continue
            hhmmss, kind, realtime, ext = m.groups()
            rid = f'{day.name}/demo_{hhmmss}'
            r = runs.setdefault(rid, {'id': rid, 'date': day.name,
                                      'time': _fmt_time(hhmmss),
                                      'dir': str(day), 'stem': f'demo_{hhmmss}',
                                      'videos': {}, 'csv': None, 'logs': []})
            if ext == 'mp4' and kind in P.CHANNELS:
                # 실시간 판이 있으면 그쪽이 우선 — 무접미 판은 2.5배속으로 굳은 기록
                cur = r['videos'].get(kind)
                if cur is None or (realtime and '_realtime' not in cur):
                    r['videos'][kind] = str(f)
            elif ext == 'csv' and kind == 'state':
                r['csv'] = str(f)
            elif ext == 'log':
                r['logs'].append(str(f))
    return [runs[k] for k in sorted(runs)]


def read_series(path):
    """상태 CSV → 솎은 시계열 + 계측 요약. 컬럼 구성이 판마다 다르므로
    헤더를 보고 있는 것만 싣는다(전압은 2026-08-20 저녁부터 생겼다)."""
    if not path or not pathlib.Path(path).exists():
        return None, {}
    with open(path, newline='') as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return None, {}
    cols = [c for c in rows[0] if c and c != 't_s']
    stride = max(1, int(P.SERIES_STRIDE_S / max(1e-6, _dt(rows))))
    out = {'t': []}
    for c in cols:
        out[c] = []
    for i in range(0, len(rows), stride):
        r = rows[i]
        out['t'].append(round(float(r['t_s']), 2))
        for c in cols:
            v = r.get(c, '')
            try:
                out[c].append(round(float(v), 2))
            except (TypeError, ValueError):
                out[c].append(None)

    joints = P.ARM_ORDER + [P.GRIPPER]
    temps = [c for c in cols if c.endswith('_temp')]
    volts = [c for c in cols if c.startswith('volt')]
    grip = out.get(f'{P.GRIPPER}_deg') or []
    summary = {
        'seconds': out['t'][-1] if out['t'] else 0.0,
        'samples': len(rows),
        'joints': [j for j in joints if f'{j}_deg' in out],
        'temp_max': _max_of(out, temps),
        'temp_max_at': _argmax_t(out, temps),
        'volt_min': _min_of(out, volts) if volts else None,
        'grip_min': min([g for g in grip if g is not None], default=None),
        'torque_off_at': _first_zero_t(out.get('torque'), out['t']),
        'closes': _close_events(grip, out['t']),
    }
    return out, summary


def _dt(rows):
    try:
        return float(rows[1]['t_s']) - float(rows[0]['t_s'])
    except Exception:
        return 0.05


def _vals(series, cols):
    return [v for c in cols for v in series.get(c, []) if v is not None]


def _max_of(series, cols):
    v = _vals(series, cols)
    return max(v) if v else None


def _min_of(series, cols):
    v = _vals(series, cols)
    return min(v) if v else None


def _argmax_t(series, cols):
    best = (None, None)
    for c in cols:
        for t, v in zip(series['t'], series.get(c, [])):
            if v is not None and (best[0] is None or v > best[0]):
                best = (v, t)
    return best[1]


def _first_zero_t(torque, ts):
    if not torque:
        return None
    for t, v in zip(ts, torque):
        if v == 0:
            return t
    return None


def _close_events(grip, ts):
    """열림→닫힘 전이 시각 — 파지 시도의 위치를 시간축에 찍는다."""
    out, prev = [], None
    for t, g in zip(ts, grip):
        if g is None:
            continue
        if prev is not None and prev >= P.GRIP_CLOSED_DEG and g < P.GRIP_CLOSED_DEG:
            out.append(round(t, 1))
        prev = g
    return out


def grab_frames(path, fps, max_w):
    """mp4 → [{'t', 'b'(base64 JPEG)}] — 원본을 통째로 싣지 않고 다시 만든다."""
    import av
    from PIL import Image
    out = []
    with av.open(path) as c:
        st = c.streams.video[0]
        st.thread_type = 'AUTO'
        tb = float(st.time_base or 0) or 1 / float(st.average_rate or 25)
        nxt = 0.0
        for frame in c.decode(st):
            t = (frame.pts or 0) * tb
            if t + 1e-6 < nxt:
                continue
            nxt = t + 1.0 / fps
            img = frame.to_image()
            if img.width > max_w:
                img = img.resize((max_w, max(2, round(img.height * max_w / img.width))))
            buf = io.BytesIO()
            img.convert('RGB').save(buf, 'JPEG', quality=82)
            out.append({'t': round(t, 3),
                        'b': base64.b64encode(buf.getvalue()).decode()})
    return out


def read_datasets():
    """LeRobot 데이터셋 목록 — 차량 호환 여부를 함께 표시한다."""
    root = pathlib.Path(P.DATASET_ROOT)
    out = []
    if not root.exists():
        return out
    for info_p in sorted(root.glob('*/meta/info.json')):
        try:
            info = json.loads(info_p.read_text())
        except Exception:
            continue
        repo_id = info_p.parent.parent.name
        features = sorted(info.get('features', {}).keys())
        wrist_only = ('observation.images.wrist' in features
                      and 'observation.images.depth' not in features)
        out.append({'repo_id': repo_id,
                    'episodes': info.get('total_episodes'),
                    'frames': info.get('total_frames'),
                    'fps': info.get('fps'),
                    'robot_type': info.get('robot_type'),
                    'features': features,
                    'profile': 'vehicle' if repo_id.startswith('so101_car') else 'legacy',
                    'compatible': repo_id.startswith('so101_car') and wrist_only})
    return out


def read_status():
    """차량 운영 현황을 구조화된 원자료에서 다시 계산한다."""
    status = dict(P.STATUS)
    yolo = pathlib.Path(P.YOLO_LOG)
    if yolo.exists():
        rows = []
        for line in yolo.read_text().splitlines():
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
        hits = sum(r.get('target') is not None for r in rows)
        status['yolo'] = {'frames': len(rows), 'hits': hits,
                          'rate_pct': round(100 * hits / len(rows), 1) if rows else 0,
                          'confidence': 0.40, 'gate': 'blocked'}

    pick = pathlib.Path(P.PICK_LOG)
    if pick.exists():
        with pick.open(newline='') as fh:
            rows = list(csv.DictReader(fh))
        car = [r for r in rows if r.get('repo') == 'so101_car']
        if car:
            last = car[-1]
            status['pick'] = {
                'cycles': len(car),
                'success': sum(r.get('result') == 'success' for r in car),
                'last_at': last.get('ts'),
                'last_result': last.get('result'),
                'last_reason': last.get('reason'),
            }
    return status


def main():
    runs = scan_runs()
    if not runs:
        print(f'기록이 없다 — {P.MEDIA_ROOT} 아래에 demo_*.mp4 가 있어야 한다')
    show = set(getattr(P, 'SHOW', None) or [])
    # 가장 최근 시행은 항상 영상까지 싣는다 — 데모를 새로 찍으면 그 자리에서
    # 화면에 뜨는 것이 이 대시보드의 쓸모다(SHOW 를 매번 손으로 고칠 수 없다).
    if runs and getattr(P, 'SHOW_LATEST', True):
        show.add(runs[-1]['id'])
    missing = show - {r['id'] for r in runs}
    if missing:
        print(f'  ! SHOW 에 있는데 기록이 없는 시행: {sorted(missing)}')
    # 최신부터 MAX_RUNS 개, 단 **SHOW 는 오래돼도 반드시 남긴다** — 화면에 세우기로
    # 정한 시행이 최신 목록에서 밀려 조용히 빠지면 영상 없는 화면이 나온다
    keep = [r for r in runs[-P.MAX_RUNS:]]
    kept = {r['id'] for r in keep}
    keep = [r for r in runs if r['id'] in show and r['id'] not in kept] + keep
    dropped = len(runs) - len(keep)
    if dropped > 0:
        print(f'  오래된 시행 {dropped}개는 싣지 않는다 (MAX_RUNS={P.MAX_RUNS})')
    runs = keep

    # 너무 짧은 시행은 여기서 뺀다. **계측을 읽은 뒤** 판단해야 한다 — 파일이 있어도
    # 안이 비었을 수 있어(실측: demo_163812 이 0.0초·표본 0) 목록만 봐서는 모른다.
    for r in runs:
        r['series'], r['summary'] = read_series(r['csv'])
    lo = getattr(P, 'MIN_DUR_S', 0)
    if lo:
        short = [r for r in runs if (r['summary'].get('seconds') or 0) < lo]
        if short:
            print(f'  {lo:.0f}초 미만이라 뺀다: '
                  + ', '.join(f"{r['id'].split('/')[-1]}({r['summary'].get('seconds') or 0:.0f}s)"
                              for r in short))
            runs = [r for r in runs if r not in short]

    by_run = {}
    for r in runs:
        r['channels'] = [c for c in P.CHANNEL_ORDER if c in r['videos']]
        r['labels'] = {c: P.CHANNELS[c]['label'] for c in r['channels']}
        # 배속 경고는 **mpjpeg 로 받는 채널**에만 해당한다. 그 셋은 벽시계
        # 타임스탬프 없이 저장하면 2.5배속으로 굳어(2026-08-20 실측) `_realtime`
        # 판을 따로 만들었다. sim·screen 은 처음부터 실시간이라 대상이 아니다.
        r['speed_warn'] = {c: bool(P.CHANNELS[c].get('realtime')
                                   and '_realtime' not in r['videos'][c])
                           for c in r['channels']}
        r['show'] = r['id'] in show
        if r['show']:
            by_run[r['id']] = {}
            for c in r['channels']:
                w = getattr(P, 'VIDEO_W_BY_CHANNEL', {}).get(c, P.VIDEO_MAX_W)
                fr = grab_frames(r['videos'][c], P.VIDEO_FPS, w)
                by_run[r['id']][c] = fr
                print(f'  {r["id"]} {c}: {len(fr)} 프레임')

    vids, total = ({}, 0)
    if by_run:
        vids, total = V.build(by_run, P.VIDEO_FPS, P.VIDEO_BUDGET_MB,
                              crf=P.VIDEO_CRF,
                              crf_by_cam=getattr(P, 'VIDEO_CRF_BY_CHANNEL', None),
                              out_dir=getattr(P, 'VIDEO_DIR', None),
                              url_base=getattr(P, 'VIDEO_DIR', 'media'))
        print(f'  영상 {total/1e6:.1f}MB (예산 {P.VIDEO_BUDGET_MB}MB)')
    for r in runs:
        r['video'] = vids.get(r['id'], {})
        for k in ('dir', 'videos', 'csv', 'logs'):
            r.pop(k, None)                 # 남의 기계에서 열리는 화면이라 경로는 뺀다

    pathlib.Path('runs.json').write_text(
        json.dumps(runs, ensure_ascii=False, separators=(',', ':')))
    ds = read_datasets()
    pathlib.Path('datasets.json').write_text(
        json.dumps(ds, ensure_ascii=False, separators=(',', ':')))
    pathlib.Path('status.json').write_text(
        json.dumps(read_status(), ensure_ascii=False, separators=(',', ':')))
    shown = sum(1 for r in runs if r['show'])
    print(f'runs.json {len(runs)} 시행 (영상 {shown}개) · datasets.json {len(ds)} 개'
          ' · status.json 차량 현황')


if __name__ == '__main__':
    main()
