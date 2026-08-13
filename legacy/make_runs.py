"""추적 JSONL → runs.json — 대시보드가 읽는 시행 기록.

두 실험을 한 화면에 올린다.
  · **주행 파이프라인** (`trace.jsonl`) — 이동·접근까지 포함. 통 인식에 막혀 있다
  · **팔 단독** (`armonly_trace.jsonl`) — 차체 고정, 팔만으로 줍고 놓기

실패만 모아 두면 "무엇이 정상인지"의 기준이 없어 화면을 읽을 수 없다. 그래서
성공 시행을 반드시 함께 싣는다. 성공은 팔 단독 쪽에만 있다 — 주행 76시행 중
실좌표로 통에 들어간 것은 하나도 없다(그 사실 자체가 이 프로젝트의 현재 상태다).

판정은 로그가 아니라 **실좌표**다. 개구부 안(반폭 68mm)이고 벽 상단(0.090m)보다
낮아야 통 안이다.

사용: python3 make_runs.py
"""
import json
import math
import os
import pathlib

HERE = pathlib.Path(__file__).parent
DRIVE = pathlib.Path.home() / 'capstone_tools/logs/trace.jsonl'
ARM = pathlib.Path.home() / 'capstone_tools/logs/armonly_trace.jsonl'
TRASH_DEF = (0.15, 0.62)
OPEN_HALF, WALL_TOP = 0.068, 0.090
ARM_ORDER = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
             'arm_wrist_flex', 'arm_wrist_roll']
# 주행 쪽에서 실을 시행. 세 '거짓 성공'은 이 프로젝트의 핵심 발견이라 반드시 넣고,
# 파지 자체가 안 된 시행 하나를 대조로 붙인다.
DRIVE_PICK = [14, 10, 17, 13]


def rows(path):
    if not path.exists():
        return []
    return [json.loads(ln) for ln in path.read_text(encoding='utf-8').splitlines() if ln.strip()]


def step(r):
    jp = r.get('joints') or {}
    return {
        't': r.get('t'),
        'stage': r.get('stage'),
        'q': [round(jp.get(k, 0.0), 4) for k in ARM_ORDER],
        'ga': r.get('grip_ang'),
        'gl': r.get('grip_load'),
        'odom': r.get('odom'),
        'jaw': r.get('jaw'),
        'cube': r.get('cube_gz'),
        'robot': r.get('robot_gz'),
        'claim': r.get('코드주장'),
        'verdict': r.get('판정'),
        'er': r.get('er_mm'),
        'brg': r.get('brg_deg'),
    }


def finish(steps, rid, kind, trash):
    """시행 하나를 대시보드 레코드로. 실좌표로 판정한다."""
    steps = [s for s in steps if s['ga'] is not None and s['jaw']]
    if len(steps) < 3:
        return None
    # 기록 순서가 곧 시간 순서는 아니다 — 복구 사이클이 끼면 파일에 뒤섞여 들어간다
    # (실측: 주행 #13이 212.1 → 168.4 → 217.8). 타임라인은 단조여야 하므로 t로 세운다.
    steps.sort(key=lambda s: s['t'])
    t0 = steps[0]['t']
    for s in steps:
        s['t'] = round(s['t'] - t0, 1)
    cubes = [s['cube'] for s in steps if s['cube']]
    last = cubes[-1] if cubes else None
    dist = truth = None
    if last:
        dx, dy = last[0] - trash[0], last[1] - trash[1]
        dist = round(math.hypot(dx, dy) * 1000)
        truth = bool(abs(dx) < OPEN_HALF and abs(dy) < OPEN_HALF and last[2] < WALL_TOP)
    claim = next((s['claim'] for s in reversed(steps) if s['claim']), None)
    return {
        'id': rid,
        'kind': kind,
        'dur': steps[-1]['t'],
        'steps': steps,
        'claim': claim,
        'cube': last,
        'truth': truth,
        'dist': dist,
        'trash': [round(trash[0], 4), round(trash[1], 4)],
        # 물체 크기는 바닥에 놓였을 때의 중심 높이에서 나온다 — 무대가 3cm/4cm를
        # 오갔기 때문에 상수로 박으면 게이지의 기준선이 틀린다.
        'cube_size': round(cubes[0][2] * 2, 3) if cubes else None,
    }


def from_drive():
    out, cur, rid = [], [], 0
    for r in rows(DRIVE):
        if r.get('stage') == '시작' and cur:
            rec = finish([step(x) for x in cur], rid, '주행', TRASH_DEF)
            if rec and rid in DRIVE_PICK:
                out.append(rec)
            rid += 1
            cur = []
        cur.append(r)
    if cur:
        rec = finish([step(x) for x in cur], rid, '주행', TRASH_DEF)
        if rec and rid in DRIVE_PICK:
            out.append(rec)
    out.sort(key=lambda r: DRIVE_PICK.index(r['id']))
    return out


def from_arm():
    out, by = [], {}
    for r in rows(ARM):
        by.setdefault(r.get('trial', 0), []).append(r)
    for trial, rs in sorted(by.items()):
        trash = next((r['trash_gz'][:2] for r in reversed(rs) if r.get('trash_gz')), None)
        if trash is None:
            # 통 좌표가 없으면 판정 기준이 없다. 억지로 기본값을 쓰면 다른 무대의
            # 통을 기준으로 재게 되므로 통째로 버린다.
            print(f'  팔 단독 시행 {trial}: 통 좌표 없음 — 제외')
            continue
        rec = finish([step(x) for x in rs], f'A{trial}', '팔 단독', tuple(trash))
        if rec:
            out.append(rec)
    return out


def main():
    drive, arm = from_drive(), from_arm()
    runs = arm + drive
    if not runs:
        raise SystemExit('실을 시행이 없다')
    (HERE / 'runs.json').write_text(
        json.dumps(runs, ensure_ascii=False, separators=(',', ':')), encoding='utf-8')
    print(f'runs.json {os.path.getsize(HERE / "runs.json") // 1024}KB · 시행 {len(runs)}')
    for r in runs:
        mark = '○ 통 안' if r['truth'] else ('✗ 통 밖' if r['truth'] is False else '? 판정불가')
        print(f'  #{str(r["id"]):4s} {r["kind"]:6s} {mark:8s} '
              f'통까지 {str(r["dist"]) + "mm":>7s} · 주장 {str(r["claim"]):12s} · '
              f'물체 {r["cube_size"]}m · {len(r["steps"])}단계 · {r["dur"]}s')


if __name__ == '__main__':
    main()
