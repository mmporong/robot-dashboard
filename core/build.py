"""템플릿 + 데이터 JSON → 완성된 한 장짜리 HTML.

데이터를 프롬프트로 옮기지 않고 파일에서 읽어 치환한다. 수십 KB JSON을 손으로 옮기면
따옴표 하나에 통째로 깨진다.

무엇을 어느 자리에 넣을지는 `project.py`의 `DATA`가 정한다 — 프로젝트마다 화면이
다르므로 자리 이름도 다르다.
"""
import json
import pathlib

import project as P

HERE = pathlib.Path.cwd()


def main():
    tpl = (HERE / P.TEMPLATE).read_text(encoding='utf-8')
    subs = {slot: json.loads((HERE / name).read_text(encoding='utf-8'))
            for slot, name in P.DATA.items()}
    out = tpl
    for slot, value in subs.items():
        out = out.replace(slot, json.dumps(value, ensure_ascii=False, separators=(',', ':')))

    missing = [s for s in subs if s in out]
    if missing:
        raise SystemExit(f'치환 실패: {missing}')

    dest = HERE / 'dashboard.html'
    dest.write_text(out, encoding='utf-8')
    sizes = ' · '.join(f'{pathlib.Path(n).stem} {len(v)}'
                       for (s, n), v in zip(P.DATA.items(), subs.values()))
    print(f'{dest.name} {len(out.encode()) // 1024}KB  ({sizes})')


if __name__ == '__main__':
    main()
