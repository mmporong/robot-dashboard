"""인박스 전량 → `trend.json` — 시행이 쌓이면서 무엇이 나빠졌나.

한 시행을 되감아 보는 것과 **시행들을 견주는 것**은 다른 일이다. 평가를 *돌리는* 쪽은
이미 무료 오픈소스가 채웠지만(LeRobot `lerobot-eval`, AllenAI vla-evaluation-harness),
**실행 간 비교·회귀 탐지**는 그 선도 하네스에서도 아직 Planned Features다
(`Reference score comparison: Regression testing against known model+benchmark scores`,
2026-08-13 확인 · 코드 검색 `reference_score` 0건). 여기가 그 자리다.

**세 가지를 지킨다.**

1. **세대(epoch)를 넘어 비교하지 않는다.** 무대 배치가 바뀌면 그 전후는 같은 잣대가
   아니다. 세대는 파일 메타데이터에서 읽는다 — id로 짐작하지 않는다.
2. **성공률만 보지 않는다.** 이 기록에서 실제로 일어난 사고는 성공률 하락이 아니라
   **판정 근거의 소실**이었다(어느 시점부터 큐브 실좌표가 통째로 안 남았다).
   성공률만 보면 "시행이 줄었다"로 읽히고 조용히 지나간다.
3. **탐지기를 먼저 의심한다.** 새 판정 기준은 *아무 의미 없는 것*도 통과시키는지
   먼저 돌려 봐야 한다. 순열 검정으로 유의확률을 붙이고, 순수 잡음에 대해 거짓양성률이
   유의수준과 맞는지 자체 시험한다(`--selftest`).

사용: python3 trend.py [입력디렉터리]
      python3 trend.py --selftest      # 탐지기가 잡음에 안 걸리는지
"""
import json
import os
import pathlib
import random
import statistics
import sys

import project as P

# 프로젝트 리더. `run_of(path) → (메타, 시행)` 하나만 쓴다 — 그 이상 들여다보면
# 프로젝트마다 다른 `steps_of`·`judge`에 묶여 이 파일이 중립일 수 없다.
import mcap_read as R

HERE = pathlib.Path.cwd()          # 생성물은 프로젝트 폴더에 떨어진다
INBOX = pathlib.Path.home() / 'capstone_tools/mcap'
OUT = HERE / 'trend.json'

# 순열 검정 설정. 씨앗을 고정해 같은 입력이면 같은 결론이 나오게 한다 —
# 돌릴 때마다 유의확률이 흔들리면 "회귀 알림"을 믿을 수 없다.
TRIALS, ALPHA, MIN_SIDE, SEED = 4000, 0.05, 5, 20260813

# 기록되지 않은 시도. **분모 문제**다 — 무대 배치 단계에서 끊긴 시행은 추적 파일에
# 한 줄도 안 남아서, 기록만 세면 성공률이 실제보다 높게 나온다. 아는 것만 적고
# 출처를 남긴다. 모르는 세대는 비워 두고 화면에도 "미상"으로 표시한다.
ATTEMPTS, EPOCH_NOTE = P.ATTEMPTS, P.EPOCH_NOTE

METRICS = [
    ('judged', '판정 가능', '큐브 실좌표가 남아 판정할 수 있었나', ''),
    ('false_ok', '거짓 성공', '코드는 성공을 주장했는데 실좌표는 통 밖', ''),
    ('truth', '실좌표 성공', '물체가 실제로 통 안에 들어갔나', ''),
    # 성공률이 0이나 1에 붙어 있으면 아무 말도 못 한다. 거리는 그 사이에서 계속 움직여
    # "성공은 못 했지만 가까워지고 있나"를 볼 수 있는 유일한 지표다.
    ('dist', '통까지 거리', '물체가 통 중심에서 얼마나 멀리 남았나', 'mm'),
]


def order_of(rid):
    """파일 이름 끝의 번호. 시행 순서는 세대 안에서만 뜻이 있다."""
    tail = ''.join(c for c in rid if c.isdigit())
    return int(tail) if tail else 0


def split_gap(xs, min_side):
    """앞뒤 평균이 가장 크게 갈리는 자리와 그 차이. 없으면 (0, None)."""
    n = len(xs)
    if n < 2 * min_side:
        return 0.0, None
    total, left, best = sum(xs), 0.0, (0.0, None)
    for i in range(1, n):
        left += xs[i - 1]
        if i < min_side or n - i < min_side:
            continue
        d = abs(left / i - (total - left) / (n - i))
        if d > best[0]:
            best = (d, i)
    return best


def change_point(xs, trials=TRIALS, min_side=MIN_SIDE, seed=SEED):
    """평균이 갈리는 자리 하나와 유의확률.

    **순서를 섞으면 사라지는가**를 묻는다. 관측된 최대 격차를, 순서만 무작위로 바꾼
    같은 값들에서 다시 잰 격차 분포와 견준다. 최대값 통계를 쓰므로 후보 경계를 여럿
    들여다본 것에 대한 보정이 이미 들어 있다 — 따로 보정하지 않아도 된다.

    p는 (초과 횟수+1)/(시행+1)이다. +1이 없으면 0회일 때 p=0이 되어 "절대 확실"이
    되는데, 4000번 돌려 못 봤다는 것과 불가능하다는 것은 다르다.
    """
    obs, at = split_gap(xs, min_side)
    if at is None:
        return None
    rng, shuf, hits = random.Random(seed), list(xs), 0
    for _ in range(trials):
        rng.shuffle(shuf)
        if split_gap(shuf, min_side)[0] >= obs:
            hits += 1
    return {'at': at, 'effect': round(obs, 4), 'p': round((hits + 1) / (trials + 1), 4),
            'before': round(statistics.fmean(xs[:at]), 4),
            'after': round(statistics.fmean(xs[at:]), 4)}


def meta_only(path):
    """세대만 먼저 본다 — 추세에서 뺄 파일까지 통째로 푸는 것은 낭비다(163MB짜리가 있다)."""
    from mcap.reader import make_reader
    with path.open('rb') as f:
        for md in make_reader(f).iter_metadata():
            if md.name == 'run':
                return dict(md.metadata)
    return {}


def collect(inbox):
    runs, skipped = [], []
    for path in sorted(inbox.glob('*.mcap')):
        meta = meta_only(path)
        if not meta.get('epoch'):
            # `ros2 bag record`가 받아 적은 파일은 세대를 모른다. 억지로 끼워 넣으면
            # 다른 무대의 시행이 같은 추세선에 섞인다 — 빼되 뺐다고 적는다.
            skipped.append((path.stem, f'{path.stat().st_size / 1e6:.0f}MB · 세대 미상'))
            continue
        _, rec = R.run_of(path)
        if rec is None:
            skipped.append((path.stem, '판정 기준 없음'))
            continue
        judged = rec['truth'] is not None
        runs.append({
            'id': path.stem, 'kind': meta['kind'], 'epoch': meta['epoch'],
            'order': order_of(path.stem), 'steps': len(rec['steps']), 'dur': rec['dur'],
            'judged': int(judged),
            'truth': int(bool(rec['truth'])) if judged else None,
            'claim': rec['claim'],
            'false_ok': int(rec['claim'] == 'PICK_SUCCESS' and not rec['truth']) if judged else None,
            'dist': rec['dist'],
        })
    runs.sort(key=lambda r: (r['epoch'], r['order']))
    return runs, skipped


def summarise(runs):
    out = []
    for key in sorted({r['epoch'] for r in runs}):
        g = [r for r in runs if r['epoch'] == key]
        j = [r for r in g if r['judged']]
        dists = [r['dist'] for r in j if r['dist'] is not None]
        out.append({
            'key': key, 'kind': g[0]['kind'], 'n': len(g), 'judged': len(j),
            'attempts': ATTEMPTS.get(key),
            'truth': sum(r['truth'] for r in j),
            'false_ok': sum(r['false_ok'] for r in j),
            'dist_med': round(statistics.median(dists)) if dists else None,
            'note': EPOCH_NOTE.get(key, ''),
        })
    return out


def findings(runs):
    """세대마다·지표마다 회귀를 찾는다. 세대를 가로질러 묻지 않는다."""
    out = []
    for key in sorted({r['epoch'] for r in runs}):
        g = [r for r in runs if r['epoch'] == key]
        for field, label, why, unit in METRICS:
            xs = [r[field] for r in g if r[field] is not None]
            ids = [r['id'] for r in g if r[field] is not None]
            head = {'epoch': key, 'metric': field, 'label': label, 'why': why,
                    'unit': unit, 'n': len(xs)}
            if len(xs) < 2 * MIN_SIDE:
                out.append(dict(head, verdict='few'))
                continue
            if len(set(xs)) == 1:
                # 표본은 충분한데 값이 하나뿐이다. "표본 부족"과 섞으면 안 된다 —
                # 이쪽은 **변화가 없다는 사실 자체가 결론**이다(주행 40시행 통 안 0건).
                out.append(dict(head, verdict='flat', value=xs[0]))
                continue
            cp = change_point(xs)
            if cp is None:
                out.append(dict(head, verdict='few'))
                continue
            cp.update(head)
            cp.update({'at_id': ids[cp['at']], 'verdict': 'hit' if cp['p'] < ALPHA else 'none'})
            out.append(cp)
    out.sort(key=lambda f: (f.get('p', 1.0), -f.get('effect', 0)))
    return out


def selftest(trials=300, n=40, seed=SEED):
    """**귀무모형 먼저.** 아무 변화도 없는 계열에 이 탐지기가 얼마나 걸리나.

    거짓양성률이 유의수준(0.05) 근처여야 "회귀를 찾았다"는 말에 값이 생긴다. 훨씬
    높으면 그냥 잡음을 가리키는 장치이고, 훨씬 낮으면 진짜 회귀도 놓친다.
    비교를 위해 **진짜 계단이 있는 계열**도 같이 돌린다 — 잡히긴 하는지 봐야 한다.
    """
    rng = random.Random(seed)
    for name, make in (
        ('변화 없음 (동전 던지기 40회)', lambda: [rng.randint(0, 1) for _ in range(n)]),
        ('변화 없음 (전부 0)', lambda: [0] * n),
        ('진짜 계단 (앞 20개 1 → 뒤 20개 0)', lambda: [1] * (n // 2) + [0] * (n - n // 2)),
        ('약한 계단 (0.7 → 0.3)', lambda: [int(rng.random() < 0.7) for _ in range(n // 2)]
                                          + [int(rng.random() < 0.3) for _ in range(n - n // 2)]),
    ):
        hits = 0
        for k in range(trials):
            cp = change_point(make(), trials=400, seed=seed + k)
            if cp and cp['p'] < ALPHA:
                hits += 1
        print(f'  {name:34s} p<{ALPHA} 비율 {hits / trials:5.1%}  ({hits}/{trials})')


def selftest_main():
    print(f'탐지기 자체 시험 — 순열 검정 · 유의수준 {ALPHA}')
    selftest()


def main():
    runs, skipped = collect(INBOX)
    if not runs:
        raise SystemExit(f'세대가 적힌 MCAP이 없다: {INBOX}')
    epochs, found = summarise(runs), findings(runs)
    OUT.write_text(json.dumps({'runs': runs, 'epochs': epochs, 'findings': found,
                               'alpha': ALPHA, 'trials': TRIALS},
                              ensure_ascii=False, separators=(',', ':')), encoding='utf-8')
    print(f'{OUT.name} {os.path.getsize(OUT) // 1024}KB · 시행 {len(runs)} · 세대 {len(epochs)}')
    for s in skipped:
        print(f'  추세에서 제외: {s[0]} — {s[1]}')
    print()
    for e in epochs:
        att = f'/{e["attempts"]} 시도' if e['attempts'] else ''
        print(f'  [{e["key"]:9s}] {e["kind"]:6s} 기록 {e["n"]}{att} · 판정 가능 {e["judged"]}'
              f' · 통 안 {e["truth"]} · 거짓 성공 {e["false_ok"]}'
              f' · 통까지 중앙 {e["dist_med"]}mm')
    print()
    for f in found:
        tag, u = f'  [{f["epoch"]:9s}] {f["label"]:10s}', f.get('unit', '')
        if f['verdict'] == 'few':
            print(f'{tag} 시행 {f["n"]}개 — 표본 부족, 판단 보류')
        elif f['verdict'] == 'flat':
            print(f'{tag} {f["n"]}시행 내내 {f["value"]}{u} — 변한 적이 없다')
        elif f['verdict'] == 'hit':
            print(f'{tag} ⚠ {f["at_id"]}부터 {f["before"]:.2f}{u} → {f["after"]:.2f}{u} · p={f["p"]}')
        else:
            print(f'{tag} 변화 없음 (가장 큰 격차 {f["effect"]:.2f}{u} · p={f["p"]})')


if __name__ == '__main__':
    main()
