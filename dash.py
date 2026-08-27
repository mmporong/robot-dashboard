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
    # 필수 키는 프로젝트가 선언할 수 있다 — 아래 기본값은 ROS/MCAP 기록을 읽는
    # 프로젝트의 것이라, 실기 기록(mp4+CSV)을 읽는 프로젝트에는 TF 프레임처럼
    # 존재하지 않는 항목이 섞여 있다 (2026-08-21 so101-arm 추가).
    need = getattr(P, 'REQUIRED', None) or [
        'ARM_ORDER', 'GRIPPER', 'JAW_FRAME', 'BASE_FRAME', 'CAMS', 'judge',
        'SHOW', 'TEMPLATE', 'DATA', 'FRAME_BUDGET_MB']
    miss = [k for k in need if not hasattr(P, k)]
    if miss:
        raise SystemExit(f'project.py에 없는 것: {", ".join(miss)}')
    bad = [n for n in [P.TEMPLATE] if not (home / n).exists()]
    if bad:
        raise SystemExit(f'파일이 없다: {", ".join(bad)}')
    cams = getattr(P, 'CAMS', None) or getattr(P, 'CHANNELS', {})
    if cams and isinstance(next(iter(cams.values())), dict):
        ncam = sum(not value.get('legacy', False) for value in cams.values())
        channel_label = '활성 채널'
    else:
        ncam = len(set(cams.values())) if cams else 0
        channel_label = '카메라'
    print(f'설정 온전 · 관절 {len(P.ARM_ORDER)}+그리퍼 · {channel_label} {ncam}종'
          f' · 화면에 세울 시행 {len(getattr(P, "SHOW", []) or [])}개')
    for slot, name in P.DATA.items():
        mark = '있음' if (home / name).exists() else '없음 — read/trend를 먼저 돌릴 것'
        print(f'  {slot:18s} → {name:18s} {mark}')
    if getattr(P, 'REQUIRE_FRESH_BUILD', False):
        inputs = [home / P.TEMPLATE] + [home / n for n in P.DATA.values()]
        missing = [p.name for p in inputs if not p.exists()]
        out = home / 'dashboard.html'
        if missing:
            raise SystemExit(f'대시보드 입력이 없다: {", ".join(missing)} — read를 먼저 실행')
        if not out.exists():
            raise SystemExit('dashboard.html이 없다 — build를 실행')
        newer = [p.name for p in inputs if p.stat().st_mtime > out.stat().st_mtime]
        if newer:
            raise SystemExit(f'dashboard.html이 입력보다 오래됐다: {", ".join(newer)} — build를 다시 실행')
        print('  dashboard.html 최신성 PASS')


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
        # 누적·회귀는 시행이 여러 판(epoch)으로 쌓이는 프로젝트의 것이다.
        # `TREND = False` 인 프로젝트에서 all 을 돌 때 조용히 건너뛴다.
        if step == 'trend' and not getattr(P, 'TREND', True):
            print('[trend] 이 프로젝트는 회귀 추적을 쓰지 않는다 — 건너뜀')
            continue
        print(f'[{step}]')
        if step == 'read':
            # 기록 형식은 프로젝트마다 다르다. MCAP(ROS bag)이 아닌 기록을 읽는
            # 프로젝트는 `read_runs.py` 를 두면 그쪽이 쓰인다 — 이름이 mcap 인
            # 모듈에 mp4·CSV 리더를 넣으면 다음 사람이 반드시 오해한다.
            if (home / 'read_runs.py').exists():
                import read_runs
                read_runs.main()
            else:
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
