#!/usr/bin/env python3
"""LeRobot 표준 데이터셋 레코더 (2026-08-21) — 패널 서버에 붙는 기록 계층.

자체 CSV 가 아니라 **LeRobotDataset** 으로 남긴다. SO-101 은 lerobot 공식
지원 로봇이라, 같은 포맷으로 모아 두면 `lerobot-train` 이 그대로 읽고
허브 업로드도 된다. feature 규격은 손으로 짜지 않고 공식 유틸
(`hw_to_dataset_features`)에 맡겨 공식 record 산출물과 구조를 맞춘다.

기록되는 것
  observation.state          6 관절 현재각 [°] (gripper 포함)
  action                     6 관절 **명령 목표각** — 패널이 명령을 낼 때마다
                             note_action() 으로 갱신되고, 명령 사이에는 마지막
                             목표를 유지한다(zero-order hold). 목표가 한 번도
                             없으면 현재각으로 채운다.
  observation.images.wrist   손목캠 (있을 때)
  observation.images.depth   뎁스캠 컬러 (있을 때)

품질 계측: 상태 갱신이 기록 주기보다 느리면 같은 값이 반복되어 '움직이지 않는
데이터'가 쌓인다. status()의 dup_pct 가 그것을 그대로 보여준다 — 학습에 쓰기
전에 이 숫자를 먼저 본다.
"""
import pathlib
import threading
import time

import numpy as np

JOINTS6 = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
           'wrist_flex', 'wrist_roll', 'gripper']
DEFAULT_ROOT = pathlib.Path.home() / 'so101_datasets'


def _decode(jpeg):
    """JPEG bytes → RGB ndarray (없으면 None)."""
    if not jpeg:
        return None
    import cv2
    a = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    return None if a is None else cv2.cvtColor(a, cv2.COLOR_BGR2RGB)


class Recorder(threading.Thread):
    """에피소드 하나를 fps 로 모아 LeRobotDataset 에 쓴다."""

    def __init__(self, worker, cam=None, dep=None):
        super().__init__(daemon=True)
        self.worker, self.cam, self.dep = worker, cam, dep
        self.lock = threading.Lock()
        self._closing = False
        self.ds = None
        self.repo_id = None
        self.task = ''
        self.fps = 10
        self.n = 0
        self.dup = 0
        self.episodes = 0
        self.t0 = None
        self.msg = '대기'
        self.err = ''
        self._action = None          # 마지막 명령 목표 {관절: °}
        self._last_state = None
        self._use = {'wrist': False, 'depth': False}

    # ---- 외부 인터페이스 ------------------------------------------------
    def note_action(self, target):
        """패널이 낸 명령 목표를 기록용으로 받아 둔다 (부분 갱신 허용)."""
        if not target:
            return
        with self.lock:
            a = dict(self._action or {})
            for k, v in target.items():
                if k in JOINTS6 and v is not None:
                    a[k] = float(v)
            self._action = a

    def start_episode(self, repo_id, task, fps=10, root=None,
                      wrist=True, depth=True):
        with self.lock:
            if self.ds is not None:
                return {'ok': False, 'msg': '이미 기록 중입니다'}
        st = self.worker.snapshot()
        if not (st.get('connected') and st.get('calibrated')):
            return {'ok': False, 'msg': '연결·캘리브 후에 기록할 수 있습니다'}
        pos = st.get('pos') or {}
        miss = [j for j in JOINTS6 if j not in pos]
        if miss:
            return {'ok': False, 'msg': f'관절 상태 없음: {miss}'}
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
            from lerobot.utils.feature_utils import (combine_feature_dicts,
                                                     hw_to_dataset_features)
        except Exception as e:
            return {'ok': False, 'msg': f'lerobot 임포트 실패: {e}'}

        obs_hw = {j: float for j in JOINTS6}
        use = {'wrist': False, 'depth': False}
        if wrist and self.cam is not None:
            self.cam.ensure()
            img = _decode(self.cam.snapshot_jpeg())
            if img is not None:
                obs_hw['wrist'] = tuple(img.shape)
                use['wrist'] = True
        if depth and self.dep is not None:
            self.dep.ensure()
            img = _decode(self.dep.snapshot_jpeg('rgb_jpeg'))
            if img is not None:
                obs_hw['depth'] = tuple(img.shape)
                use['depth'] = True
        features = combine_feature_dicts(
            hw_to_dataset_features(obs_hw, 'observation', use_video=True),
            hw_to_dataset_features({j: float for j in JOINTS6}, 'action',
                                   use_video=True))
        root = pathlib.Path(root or DEFAULT_ROOT).expanduser() / repo_id
        try:
            if (root / 'meta' / 'info.json').exists():
                ds = LeRobotDataset(repo_id, root=root)
                ds.resume()                       # 이어 붙이기 — 회차가 쌓인다
            else:
                ds = LeRobotDataset.create(repo_id, fps=int(fps),
                                           features=features, root=root,
                                           robot_type='so101_follower',
                                           use_videos=True)
        except Exception as e:
            return {'ok': False, 'msg': f'데이터셋 열기 실패: {type(e).__name__}: {e}'}
        with self.lock:
            self.ds, self.repo_id, self.task = ds, repo_id, task or 'pick and place'
            self.fps = max(1, int(fps))
            self.n = self.dup = 0
            self._last_state = None
            self._use = use
            self.t0 = time.monotonic()
            self.msg = '기록 중'
            self.err = ''
        if not self.is_alive():
            self.start()
        cams = [k for k, v in use.items() if v] or ['없음']
        return {'ok': True, 'repo_id': repo_id, 'root': str(root),
                'fps': self.fps, 'cameras': cams}

    def stop_episode(self, save=True):
        with self.lock:
            ds, n = self.ds, self.n
            self.ds = None
            self.msg = '저장 중' if save else '취소'
        if ds is None:
            return {'ok': False, 'msg': '기록 중이 아닙니다'}
        try:
            if save and n > 0:
                ds.save_episode()
                ds.finalize()
                with self.lock:
                    self.episodes = ds.num_episodes
                    self.msg = f'저장 완료 — {n} 프레임'
                return {'ok': True, 'frames': n, 'episodes': ds.num_episodes}
            ds.clear_episode_buffer()
            with self.lock:
                self.msg = '버림 (프레임 없음)' if n == 0 else '버림'
            return {'ok': True, 'frames': n, 'dropped': True}
        except Exception as e:
            with self.lock:
                self.err = f'{type(e).__name__}: {e}'
                self.msg = '저장 실패'
            return {'ok': False, 'msg': self.err}

    def status(self):
        with self.lock:
            on = self.ds is not None
            secs = round(time.monotonic() - self.t0, 1) if (on and self.t0) else 0.0
            dup_pct = round(100.0 * self.dup / self.n, 1) if self.n else 0.0
            return {'recording': on, 'repo_id': self.repo_id, 'task': self.task,
                    'frames': self.n, 'seconds': secs, 'fps': self.fps,
                    'dup_pct': dup_pct, 'episodes': self.episodes,
                    'cameras': [k for k, v in self._use.items() if v],
                    'msg': self.msg, 'err': self.err}

    def shutdown(self):
        self._closing = True
        if self.ds is not None:
            self.stop_episode(save=True)     # 켠 채 서버를 내려도 버리지 않는다

    # ---- 수집 루프 -------------------------------------------------------
    def run(self):
        while not self._closing:
            with self.lock:
                ds, fps = self.ds, self.fps
            if ds is None:
                time.sleep(0.2)
                continue
            t0 = time.monotonic()
            try:
                self._grab(ds)
            except Exception as e:
                with self.lock:
                    self.err = f'{type(e).__name__}: {e}'
                    self.msg = '프레임 실패 — 기록 중단'
                    self.ds = None
            time.sleep(max(0.0, 1.0 / fps - (time.monotonic() - t0)))

    def _grab(self, ds):
        st = self.worker.snapshot()
        pos = st.get('pos') or {}
        if not all(j in pos for j in JOINTS6):
            return                       # 연결 순단 — 프레임을 만들지 않는다
        state = [float(pos[j]) for j in JOINTS6]
        with self.lock:
            act = dict(self._action or {})
            same = (self._last_state is not None
                    and all(abs(a - b) < 1e-6 for a, b in zip(state, self._last_state)))
            self._last_state = state
            use = dict(self._use)
        frame = {
            'observation.state': np.array(state, dtype=np.float32),
            'action': np.array([float(act.get(j, pos[j])) for j in JOINTS6],
                               dtype=np.float32),
            'task': self.task,
        }
        if use['wrist']:
            img = _decode(self.cam.snapshot_jpeg())
            if img is None:
                return
            frame['observation.images.wrist'] = img
        if use['depth']:
            img = _decode(self.dep.snapshot_jpeg('rgb_jpeg'))
            if img is None:
                return
            frame['observation.images.depth'] = img
        ds.add_frame(frame)
        with self.lock:
            self.n += 1
            if same:
                self.dup += 1


def list_datasets(root=None):
    """수집된 데이터셋 목록 — 회차·프레임·태스크."""
    import json
    root = pathlib.Path(root or DEFAULT_ROOT).expanduser()
    out = []
    if not root.exists():
        return out
    for info_p in sorted(root.glob('*/meta/info.json')):
        try:
            info = json.loads(info_p.read_text())
        except Exception:
            continue
        out.append({'repo_id': info_p.parent.parent.name,
                    'episodes': info.get('total_episodes'),
                    'frames': info.get('total_frames'),
                    'fps': info.get('fps'),
                    'robot_type': info.get('robot_type'),
                    'features': sorted(info.get('features', {}).keys()),
                    'root': str(info_p.parent.parent)})
    return out


def episode_frames(repo_id, episode, root=None, stride=1, limit=1200):
    """에피소드의 관절 궤적을 [{관절:°}, ...] 로 — 미러 재생용."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    root = pathlib.Path(root or DEFAULT_ROOT).expanduser() / repo_id
    ds = LeRobotDataset(repo_id, root=root, episodes=[int(episode)])
    out = []
    for i in range(0, ds.num_frames, max(1, int(stride))):
        item = ds.get_raw_item(i) if hasattr(ds, 'get_raw_item') else ds[i]
        s = item['observation.state']
        vals = [float(v) for v in (s.tolist() if hasattr(s, 'tolist') else s)]
        out.append({j: round(v, 2) for j, v in zip(JOINTS6, vals)})
        if len(out) >= limit:
            break
    return out
