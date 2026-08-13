"""녹화 프레임 → run_frames.json — 시행별·시각별 카메라 영상.

종전 `frames.json`은 팔을 다섯 자세로 옮겨 가며 따로 찍은 **스틸 5장**이었다.
대시보드는 그중 관절값이 가장 가까운 것을 골라 붙였는데, 자세가 닮았다고 같은
순간이 아니다 — 성공한 시행에 큐브를 놓친 장면이 붙었고 5장뿐이라 재생도 끊겼다.
여기서는 시행이 도는 동안 같은 노드·같은 시계로 찍은 프레임을 시각으로 잇는다.

용량이 제약이다. 아티팩트 한 장에 다 들어가야 하므로 예산을 넘으면 **솎아낸다.**
솎아낸 사실은 조용히 넘기지 않고 찍는다 — 화면이 실제보다 촘촘해 보이면 안 된다.

사용: python3 make_frames.py [예산MB]
"""
import json
import math
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).parent
SRC = pathlib.Path.home() / 'capstone_tools/logs/armonly_frames.json'
OUT = HERE / 'run_frames.json'
BUDGET_MB = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0


def thin(seq, keep):
    """시각이 고르게 남도록 솎는다. 앞뒤를 자르면 시작·끝 장면이 통째로 사라진다."""
    if keep >= len(seq):
        return seq
    step = len(seq) / keep
    return [seq[min(len(seq) - 1, int(i * step))] for i in range(keep)]


def main():
    if not SRC.exists():
        raise SystemExit(f'녹화가 없다: {SRC}')
    raw = json.loads(SRC.read_text(encoding='utf-8'))

    # 시행별로 가른다. 추적의 시행 번호와 같은 값이라 runs.json의 A1·A2·A3에 붙는다.
    runs = {}
    for cam in ('front', 'wrist'):
        for f in raw.get(cam, []):
            rid = f'A{f["trial"]}'
            runs.setdefault(rid, {'front': [], 'wrist': []})[cam].append(
                {'t': f['t'], 'b': f['b']})
    for r in runs.values():
        for cam in r:
            r[cam].sort(key=lambda f: f['t'])

    # 시행마다 t를 0부터 세도록 맞춘다 — runs.json도 첫 기록을 0으로 잡는다
    for rid, r in runs.items():
        t0 = min([f['t'] for cam in r for f in r[cam]] or [0])
        for cam in r:
            for f in r[cam]:
                f['t'] = round(f['t'] - t0, 2)

    total = sum(len(f['b']) for r in runs.values() for cam in r for f in r[cam])
    budget = BUDGET_MB * 1024 * 1024
    if total > budget:
        # 손목캠을 촘촘히 남긴다. 파지·운반·놓기가 전부 거기서 벌어지고, 전방캠은
        # 파지 거리에서 팔에 가려 몇 초씩 같은 그림이다 — 같은 비율로 깎으면
        # 정작 봐야 할 쪽이 끊긴다.
        w = {'wrist': 1.0, 'front': 0.45}
        weighted = sum(len(f['b']) * w[cam] for r in runs.values() for cam in r for f in r[cam])
        k = budget / weighted
        print(f'  예산 초과 {total/1e6:.1f}MB > {BUDGET_MB}MB — 솎아낸다 '
              f'(손목 {min(1,k)*100:.0f}% · 전방 {min(1,k*w["front"])*100:.0f}%)')
        for rid, r in runs.items():
            for cam in r:
                before = len(r[cam])
                r[cam] = thin(r[cam], max(2, int(before * min(1.0, k * w[cam]))))
                print(f'    {rid} {cam}: {before} → {len(r[cam])}장')

    OUT.write_text(json.dumps(runs, ensure_ascii=False, separators=(',', ':')),
                   encoding='utf-8')
    print(f'{OUT.name}  {os.path.getsize(OUT)//1024}KB')
    for rid in sorted(runs):
        r = runs[rid]
        for cam in ('front', 'wrist'):
            if not r[cam]:
                continue
            ts = [f['t'] for f in r[cam]]
            gap = (ts[-1] - ts[0]) / max(1, len(ts) - 1)
            kb = sum(len(f['b']) for f in r[cam]) // 1024
            print(f'  {rid} {cam:5s} {len(r[cam]):4d}장 · {ts[0]:.1f}~{ts[-1]:.1f}s '
                  f'· 평균 간격 {gap:.2f}s · {kb}KB')


if __name__ == '__main__':
    main()
