"""진입점. `python3 dash.py <프로젝트> <할 일>`

    python3 dash.py capstone-pick read      MCAP → runs.json · run_frames.json
    python3 dash.py capstone-pick trend     MCAP 전량 → trend.json (누적·회귀)
    python3 dash.py capstone-pick build     템플릿 + JSON → dashboard.html
    python3 dash.py capstone-pick all       위 셋을 순서대로
    python3 dash.py capstone-pick check     프로젝트 설정이 온전한지
    python3 dash.py capstone-pick probe <파일.mcap>   이 파일에서 뭐가 읽히나
    python3 dash.py capstone-pick selftest  회귀 탐지기 귀무모형 시험

`core/`는 로봇도 과제도 모른다. 아는 것은 넘겨받은 `projects/<이름>/project.py` 하나뿐이라,
다른 로봇으로 옮길 때 고칠 곳도 그 파일 하나다. LeRobot 내보내기는 lerobot이 깔린
파이썬이 따로 필요해서 여기 두지 않는다(`projects/<이름>/README.md` 참고).
"""
import os
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
CORE, PROJECTS = ROOT / 'core', ROOT / 'projects'
JOBS = ('read', 'trend', 'build', 'all', 'check', 'probe', 'selftest')


def usage(msg=''):
    have = sorted(p.name for p in PROJECTS.iterdir()
                  if p.is_dir() and (p / 'project.py').exists())
    raise SystemExit(f'{msg}\n사용: python3 dash.py <프로젝트> <{"|".join(JOBS)}>\n'
                     f'  프로젝트: {", ".join(have) or "없음"}')


def check(P, home):
    """프로젝트 설정이 실제로 쓸 수 있는 상태인지. 없는 것을 **미리** 말해 준다."""
    need = ['ARM_ORDER', 'GRIPPER', 'JAW_FRAME', 'BASE_FRAME', 'CAMS', 'judge',
            'SHOW', 'TEMPLATE', 'DATA', 'FRAME_BUDGET_MB']
    miss = [k for k in need if not hasattr(P, k)]
    if miss:
        raise SystemExit(f'project.py에 없는 것: {", ".join(miss)}')
    bad = [n for n in [P.TEMPLATE] if not (home / n).exists()]
    if bad:
        raise SystemExit(f'파일이 없다: {", ".join(bad)}')
    print(f'설정 온전 · 관절 {len(P.ARM_ORDER)}+그리퍼 · 카메라 {len(set(P.CAMS.values()))}종'
          f' · 화면에 세울 시행 {len(P.SHOW)}개')
    for slot, name in P.DATA.items():
        mark = '있음' if (home / name).exists() else '없음 — read/trend를 먼저 돌릴 것'
        print(f'  {slot:18s} → {name:18s} {mark}')


def main():
    if len(sys.argv) < 3 or sys.argv[2] not in JOBS:
        usage('인자가 모자라거나 모르는 할 일이다.' if len(sys.argv) > 1 else '')
    name, job = sys.argv[1], sys.argv[2]
    home = PROJECTS / name
    if not (home / 'project.py').exists():
        usage(f'프로젝트 "{name}"이 없다.')

    # 프로젝트 디렉터리가 먼저다 — core 모듈이 `import project`로 그 파일을 집는다.
    # 생성물도 그 디렉터리에 떨어지도록 작업 위치를 옮긴다.
    sys.path.insert(0, str(home))
    sys.path.insert(1, str(CORE))
    os.chdir(home)
    import project as P

    if job == 'check':
        return check(P, home)
    if job == 'selftest':
        import regress
        return regress.selftest_main()
    if job == 'probe':
        import mcap_read
        return mcap_read.probe_main(sys.argv[3:])

    todo = ['read', 'trend', 'build'] if job == 'all' else [job]
    for step in todo:
        print(f'[{step}]')
        if step == 'read':
            import mcap_read
            mcap_read.main()
        elif step == 'trend':
            import regress
            regress.main()
        else:
            import build
            build.main()


if __name__ == '__main__':
    main()
