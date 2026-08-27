#!/usr/bin/env python3
"""Legacy runtime entrypoint를 canonical sibling checkout으로 넘긴다."""
import os
import pathlib
import sys


def canonical_root():
    configured = os.environ.get('SO101_CANONICAL_DIR')
    if configured:
        return pathlib.Path(configured).expanduser().resolve()
    dashboard_root = pathlib.Path(__file__).resolve().parents[3]
    return (dashboard_root.parent / 'so101-mobile-manipulation').resolve()


def redirect_if_main(module_name, relative_path):
    if module_name != '__main__':
        raise ImportError(
            f'legacy module import 차단: {relative_path}; '
            'SO101_CANONICAL_DIR의 정본 모듈을 import하세요')
    relative = pathlib.PurePosixPath(relative_path)
    if relative.is_absolute() or '..' in relative.parts:
        raise SystemExit(f'legacy redirect 경로 거부: {relative_path}')
    root = canonical_root()
    target = (root / relative).resolve()
    if not target.is_relative_to(root):
        raise SystemExit(f'canonical checkout 밖 redirect 거부: {target}')
    if not target.is_file():
        raise SystemExit(
            f'legacy 실행 차단 — canonical 진입점 없음: {target}\n'
            'SO101_CANONICAL_DIR을 올바른 checkout으로 지정하세요')
    if target.resolve() == pathlib.Path(sys.argv[0]).resolve():
        raise SystemExit('legacy redirect 순환을 감지했습니다')
    program = '/usr/bin/env'
    command = ['bash', str(target)] if target.suffix == '.sh' else [sys.executable, str(target)]
    os.execv(program, [program, *command, *sys.argv[1:]])


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit('사용법: _canonical_redirect.py <canonical-relative-path> [args...]')
    relative, sys.argv = sys.argv[1], [sys.argv[0], *sys.argv[2:]]
    redirect_if_main('__main__', relative)
