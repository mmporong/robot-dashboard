"""MCAP → LeRobotDataset v3.0. 학습 생태계에 **붙는다.**

자체 포맷을 하나 더 만드는 대신 이미 표준이 된 스키마로 나간다. 그런데 스키마를 손으로
재현하면 그것도 결국 자체 구현이라, **`lerobot` 라이브러리의 쓰기 API를 그대로 쓴다**
(`LeRobotDataset.create` → `add_frame` → `save_episode` → `finalize`). 포맷이 구성상 맞고,
lerobot이 v3.1로 올라가면 여기도 따라 올라간다.

실행에는 lerobot이 깔린 파이썬이 필요하다:

    ~/miniforge3/envs/lerobot/bin/python to_lerobot.py [--epoch arm-v1] [--out 경로]

**무엇을 싣나** — 전부 lerobot의 예약 키다(`lerobot/utils/constants.py`).

| 키 | 내용 |
|---|---|
| `observation.state` | 팔 5축 + 그리퍼, 라디안 |
| `action` | **다음 시점의 관절값** — 아래 주의 |
| `observation.environment_state` | 큐브 실좌표 xyz (Gazebo) |
| `observation.images.front` `.wrist` | 두 카메라, MP4 |
| `next.success` | **실좌표로 판정한 프레임별 성공** — 물체가 개구부 안이고 벽 상단보다 낮은가 |
| `next.reward` | 위 성공의 0/1 |

**`action`에 대한 주의.** 이 기록에는 지령값이 없다 — 관절 *측정값*만 남아 있다. 그래서
다음 시점의 측정값을 액션으로 쓴다(위치 제어 팔에서 흔히 쓰는 변환이다). 지령을 기록한
척하지 않기 위해 여기 적어 두고, 데이터셋 카드에도 같은 문장을 넣는다.

**`next.success`가 이 데이터셋의 값어치다.** LeRobot v0.6.0이 `lerobot.rewards`로 성공
판정 모델(Robometer·TOPReward 등)을 통합했는데, 그 모델들이 영상에서 *추정하려는* 값이
여기엔 시뮬레이터 실좌표로 들어 있다. 추정과 맞대 볼 정답이 붙어 나가는 셈이다.

**샘플링.** LeRobotDataset은 고정 주기를 요구하는데 이 기록은 사건 기반이다(단계가 바뀔
때만 남는다). 그래서 **카메라 시각을 시간축으로 삼고** 관절·실좌표는 영차 유지(ZOH)로
채운다 — 없는 값을 보간해 만들지 않는다. 세대마다 데이터셋을 따로 만든다: 무대가 바뀌면
같은 잣대가 아니라 한 데이터셋에 섞으면 안 된다.
"""
import argparse
import io
import pathlib
import shutil
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import project as P                                               # noqa: E402
import mcap_read as R                                             # noqa: E402
from lerobot.datasets.lerobot_dataset import LeRobotDataset      # noqa: E402

JOINTS = P.ARM_ORDER + [P.GRIPPER]
CAMS = tuple(dict.fromkeys(P.CAMS.values()))
TASK, ROBOT = P.TASK, P.ROBOT_TYPE
# 카드에 남길 문장. 코드 주석에만 적어 두면 데이터셋만 받은 사람은 못 본다.
CAVEAT = P.CAVEAT


def even(n):
    """영상 인코더는 홀수 변을 싫어한다."""
    return n - (n % 2)


def decode(b64, size=None):
    img = Image.open(io.BytesIO(R.base64.b64decode(b64))).convert('RGB')
    if size and img.size != size:
        img = img.resize(size)
    return np.asarray(img, dtype=np.uint8)


def hold(series, t):
    """t 이하에서 가장 늦은 값. 없으면 None — 앞으로 당겨 채우지 않는다."""
    best = None
    for ts, v in series:
        if ts <= t + 1e-6:
            best = v
        else:
            break
    return best


def natural_fps(frames, floor=1):
    """카메라 시각에서 고정 주기를 뽑는다. 정수 fps라야 LeRobot이 받는다.

    두 카메라를 **합쳐** 잰다. 번갈아 찍히므로 합집합이 촘촘하고, 그 위에서 각 카메라를
    영차 유지하면 프레임이 겹칠 뿐 **버려지는 프레임은 없다**. 반대로 한쪽 주기에 맞추면
    다른 쪽 프레임이 격자 사이로 빠진다 — 데이터셋에서 버리는 쪽이 더 나쁘다.
    """
    ts = sorted({round(f['t'], 3) for cam in frames for f in frames[cam]})
    if len(ts) < 2:
        return None
    gaps = sorted(b - a for a, b in zip(ts, ts[1:]) if b > a)
    return max(floor, round(1.0 / gaps[len(gaps) // 2]))


def load_episode(path):
    """MCAP 하나 → 판정된 시행과 프레임. 주기·해상도는 **여기서 정하지 않는다** —
    세대 안에서 하나로 맞춰야 하므로 세대를 다 본 뒤에 정한다."""
    meta, series, frames = R.load(path)
    rec = R.judge(path.stem, meta, series, R.steps_of(series))
    if rec is None:
        return None
    # 화면과 같은 기준으로 0초를 옮긴다 — 무대를 세우는 앞부분을 떼어 낸다
    R.rebase(rec, frames)
    has_cam = any(frames.values())
    size = None
    if has_cam:
        b = next(f['b'] for c in frames for f in frames[c])
        w, h = Image.open(io.BytesIO(R.base64.b64decode(b))).size
        size = (even(w), even(h))
    return {'id': path.stem, 'epoch': meta['epoch'], 'truth': rec['truth'],
            'rec': rec, 'frames': frames, 'size': size,
            'fps': natural_fps(frames) if has_cam else None}


def build_rows(ep, fps, size):
    """정해진 격자 위에 시행을 편다. 없는 값은 만들지 않고 그 시점을 통째로 뺀다."""
    rec, frames = ep['rec'], ep['frames']
    cams = [c for c in CAMS if frames.get(c)] if size else []
    cam_series = {c: [(f['t'], f['b']) for f in sorted(frames[c], key=lambda f: f['t'])]
                  for c in cams}
    q_series = [(s['t'], s['q'] + [s['ga'] if s['ga'] is not None else 0.0]) for s in rec['steps']]
    c_series = [(s['t'], s['cube']) for s in rec['steps'] if s['cube']]
    tx, ty = rec['trash']

    rows = []
    for i in range(int(rec['dur'] * fps) + 1):
        t = round(i / fps, 3)
        q, cube = hold(q_series, t), hold(c_series, t)
        imgs = {c: hold(cam_series[c], t) for c in cams}
        if q is None or cube is None or any(v is None for v in imgs.values()):
            continue
        ok = abs(cube[0] - tx) < R.OPEN_HALF and abs(cube[1] - ty) < R.OPEN_HALF \
            and cube[2] < R.WALL_TOP
        rows.append({'t': t, 'q': np.asarray(q, dtype=np.float32),
                     'cube': np.asarray(cube, dtype=np.float32),
                     'img': {c: decode(imgs[c], size) for c in cams},
                     'ok': bool(ok)})
    # 지령이 없으므로 다음 시점의 측정값을 액션으로 쓴다. 마지막 프레임은 다음이 없어
    # 자기 자신을 쓴다 — 그 한 프레임만 "움직이지 않으라"는 액션이 된다.
    for i, r in enumerate(rows):
        r['action'] = rows[min(i + 1, len(rows) - 1)]['q']
    return rows


def features_for(size, cams):
    vid = {}
    if size:
        w, h = size
        vid = {f'observation.images.{c}': {'dtype': 'video', 'shape': (h, w, 3),
                                           'names': ['height', 'width', 'channels']}
               for c in cams}
    return {
        'observation.state': {'dtype': 'float32', 'shape': (len(JOINTS),), 'names': JOINTS},
        'action': {'dtype': 'float32', 'shape': (len(JOINTS),), 'names': JOINTS},
        'observation.environment_state': {'dtype': 'float32', 'shape': (3,),
                                          'names': ['cube_x', 'cube_y', 'cube_z']},
        'next.success': {'dtype': 'bool', 'shape': (1,), 'names': None},
        'next.reward': {'dtype': 'float32', 'shape': (1,), 'names': None},
        **vid,
    }


def export(epoch, episodes, out_root):
    root = out_root / epoch
    if root.exists():
        shutil.rmtree(root)

    # 주기와 해상도는 **세대 안에서 하나**여야 한다. 시행마다 자연 주기가 3~4로 갈리는데,
    # 가장 빠른 쪽에 맞춰야 느린 시행이 프레임을 잃지 않는다(느린 쪽은 겹칠 뿐이다).
    rates = [e['fps'] for e in episodes if e['fps']]
    fps = max(rates) if rates else 1
    sizes = [e['size'] for e in episodes if e['size']]
    size = max(set(sizes), key=sizes.count) if sizes else None
    if sizes and len(set(sizes)) > 1:
        print(f'    해상도가 갈려 {size}로 맞춘다: {sorted(set(sizes))}')
    if size is None:
        # 카메라 없이 나가는 세대. 저차원만으로도 LeRobotDataset은 성립한다 —
        # 영상이 없다고 세대를 통째로 빠뜨리면 그 시행은 생태계 밖에 남는다.
        print('    카메라 기록이 없다 — 저차원(관절·실좌표·판정)만 내보낸다')
    cams = [c for c in CAMS if any(e['frames'].get(c) for e in episodes)] if size else []

    ds = LeRobotDataset.create(repo_id=f'jdamr/{epoch}', fps=fps, root=root,
                               features=features_for(size, cams), robot_type=ROBOT,
                               use_videos=bool(cams))
    for e in episodes:
        e['rows'] = build_rows(e, fps, size)
    # 실좌표가 없는 시행은 `observation.environment_state`를 채울 수 없어 행이 안 생긴다.
    # 조용히 빠지면 "72시행 중 40개만 나갔다"가 숫자로만 남으므로 이유와 함께 찍는다.
    thin = [e['id'] for e in episodes if len(e['rows']) < 2]
    if thin:
        print(f'    실좌표가 없어 제외 {len(thin)}건: {", ".join(sorted(thin)[:5])}'
              + (f' 외 {len(thin) - 5}건' if len(thin) > 5 else ''))
    episodes = [e for e in episodes if len(e['rows']) >= 2]
    for e in episodes:
        for r in e['rows']:
            ds.add_frame({
                'observation.state': r['q'],
                'action': r['action'],
                'observation.environment_state': r['cube'],
                'next.success': np.array([r['ok']]),
                'next.reward': np.array([1.0 if r['ok'] else 0.0], dtype=np.float32),
                **{f'observation.images.{c}': r['img'][c] for c in cams},
                'task': TASK,
            })
        ds.save_episode()
        print(f'    {e["id"]:4s} 프레임 {len(e["rows"]):4d}'
              f' · 성공 프레임 {sum(r["ok"] for r in e["rows"]):3d}'
              f' · 실좌표 판정 {"통 안" if e["truth"] else "통 밖"}')
    ds.finalize()
    (root / 'README.md').write_text(
        f'# {epoch}\n\n{TASK}\n\nrobot: {ROBOT} · fps: {fps} · episodes: {len(episodes)}'
        f' · cameras: {", ".join(cams) or "none"}\n\n**Caveats.** {CAVEAT}\n', encoding='utf-8')
    return ds, len(episodes), fps, cams


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--inbox', type=pathlib.Path,
                    default=pathlib.Path.home() / 'capstone_tools/mcap')
    ap.add_argument('--out', type=pathlib.Path,
                    default=pathlib.Path.home() / 'capstone_tools/lerobot')
    ap.add_argument('--epoch', default=None, help='이 세대만. 기본은 카메라가 있는 세대 전부')
    args = ap.parse_args()

    by_epoch, skipped = {}, []
    for path in sorted(args.inbox.glob('*.mcap')):
        if path.stat().st_size > 10_000_000:
            skipped.append(f'{path.stem}({path.stat().st_size / 1e6:.0f}MB)')
            continue
        e = load_episode(path)
        if e is None:
            skipped.append(f'{path.stem}(판정 불가)')
            continue
        if args.epoch and e['epoch'] != args.epoch:
            continue
        by_epoch.setdefault(e['epoch'], []).append(e)

    if skipped:
        print(f'  제외 {len(skipped)}건: {", ".join(skipped[:6])}'
              + (f' 외 {len(skipped) - 6}건' if len(skipped) > 6 else ''))
    if not by_epoch:
        raise SystemExit('내보낼 시행이 없다')
    for epoch, eps in sorted(by_epoch.items()):
        print(f'  [{epoch}] 시행 {len(eps)}개')
        ds, n, fps, cams = export(epoch, eps, args.out)
        print(f'    → {args.out / epoch} · 에피소드 {n} · 프레임 {ds.meta.total_frames}'
              f' · {fps}fps · 카메라 {len(cams)}')


if __name__ == '__main__':
    main()
