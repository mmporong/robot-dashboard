#!/usr/bin/env python3
"""레거시 실물 진입점이 정본 경계만 노출하는지 검증한다."""
import pathlib
import subprocess
import sys
import tempfile
import os

from test_support.redirect_contract import (
    APPROVED_OFFLINE_TEST_SHA256,
    has_exact_python_prefix, has_exact_shell_prefix,
    is_approved_offline_test, is_approved_static_tool,
)

HERE = pathlib.Path(__file__).resolve().parent
TOOLS = HERE / 'tools'
CAMERA_TOOLS = ('cam_servo.py', 'cam_calib.py')
IMPORT_BLOCKED_TOOLS = ('ik_verify.py', 'arm_lib.py', 'jog_test.py', *CAMERA_TOOLS)
HARDWARE_MARKERS = (
    'FeetechMotorsBus',
    'Torque_Enable',
    'Goal_Position',
    'write_calibration',
    'Min_Position_Limit',
    'Max_Position_Limit',
    'Protection_Current',
    'Unloading_Condition',
    'Maximum_Velocity_Limit',
    '/dev/ttyACM',
    'find_arm_port',
    'open_bus(',
)
APPROVED_STATIC_TOOLS = (
    'ds_record.py', 'log_state.py', 'sim/frame_fit.py',
    'sim/gen_ref_poses.py', 'sim/sim_core.py', 'sim/sim_view.py',
)


def run(path, *args, env=None):
    return subprocess.run([sys.executable, str(path), *args], cwd=TOOLS,
                          env=env, capture_output=True, text=True, timeout=10)


def test_imports_fail_before_stale_body():
    for name in IMPORT_BLOCKED_TOOLS:
        code = (
            'import importlib.util; '
            f'p={str(TOOLS / name)!r}; '
            's=importlib.util.spec_from_file_location("legacy_probe", p); '
            'm=importlib.util.module_from_spec(s); s.loader.exec_module(m)')
        result = subprocess.run([sys.executable, '-c', code], cwd=TOOLS,
                                capture_output=True, text=True, timeout=10)
        assert result.returncode != 0, name
        assert 'legacy module import 차단' in result.stderr, result.stderr


def test_direct_exec_preserves_args_and_uses_canonical_target():
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        env = dict(os.environ, SO101_CANONICAL_DIR=str(root))
        for name in IMPORT_BLOCKED_TOOLS:
            (root / name).write_text(
                'import json,sys; print(json.dumps(sys.argv[1:]))', encoding='utf-8')
            result = run(TOOLS / name, '--probe', name, env=env)
            assert result.returncode == 0, result.stderr
            assert result.stdout.strip() == f'["--probe", "{name}"]', result.stdout

        target = root / 'dispatcher_probe.py'
        target.write_text(
            'import json,sys; print(json.dumps(sys.argv[1:]))', encoding='utf-8')
        result = run(TOOLS / '_canonical_redirect.py', 'dispatcher_probe.py',
                     '--probe', 'dispatcher', env=env)
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == '["--probe", "dispatcher"]', result.stdout


def test_camera_force_flag_cannot_bypass_canonical_redirect():
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        env = dict(os.environ, SO101_CANONICAL_DIR=str(root))
        for name in CAMERA_TOOLS:
            wrapper = (TOOLS / name).read_text(encoding='utf-8')
            assert wrapper.splitlines() == [
                '#!/usr/bin/env python3',
                'from _canonical_redirect import redirect_if_main as _redirect',
                f"_redirect(__name__, '{name}')",
            ], name
            (root / name).write_text(
                'import json,sys; print(json.dumps(sys.argv[1:]))', encoding='utf-8')
            result = run(TOOLS / name, '--force', '--read', env=env)
            assert result.returncode == 0, result.stderr
            assert result.stdout.strip() == '["--force", "--read"]', result.stdout


def test_all_filesystem_hardware_entrypoints_are_guarded():
    """추적 전 파일도 포함해 로컬 실물 제어 구현의 재유입을 막는다."""
    offenders = []
    for path in sorted(TOOLS.rglob('*')):
        if not path.is_file() or path.suffix not in ('.py', '.sh'):
            continue
        if path.name.startswith('test_') or 'test_support' in path.parts:
            continue
        source = path.read_text(encoding='utf-8')
        if not any(marker in source for marker in HARDWARE_MARKERS):
            continue
        relative = path.relative_to(TOOLS).as_posix()
        if path.suffix == '.py':
            guarded = has_exact_python_prefix(source, relative, relative)
        else:
            guarded = has_exact_shell_prefix(source, relative, relative)
        if not guarded:
            offenders.append(str(path.relative_to(TOOLS)))
    assert not offenders, f'canonical redirect 없는 실물 진입점: {offenders}'


def test_non_redirect_tools_are_bound_to_reviewed_source():
    for relative in APPROVED_STATIC_TOOLS:
        source = (TOOLS / relative).read_text(encoding='utf-8')
        assert is_approved_static_tool(source, relative), relative
        hostile = "open('/tmp/hidden-side-effect', 'w').close()\n" + source
        assert not is_approved_static_tool(hostile, relative), relative

    offline_tests = sorted(
        path.relative_to(TOOLS).as_posix()
        for path in TOOLS.rglob('test_*.py')
        if 'test_support' not in path.parts)
    assert offline_tests == sorted(APPROVED_OFFLINE_TEST_SHA256), offline_tests
    for relative in offline_tests:
        source = (TOOLS / relative).read_text(encoding='utf-8')
        assert is_approved_offline_test(source, relative), relative
        hostile = "open('/tmp/hidden-side-effect', 'w').close()\n" + source
        assert not is_approved_offline_test(hostile, relative), relative


def test_missing_target_and_self_loop_are_blocked():
    with tempfile.TemporaryDirectory() as tmp:
        env = dict(os.environ, SO101_CANONICAL_DIR=tmp)
        missing = run(TOOLS / 'ik_verify.py', env=env)
        assert missing.returncode != 0
        assert 'canonical 진입점 없음' in missing.stderr

    env = dict(os.environ, SO101_CANONICAL_DIR=str(TOOLS))
    loop = run(TOOLS / 'ik_verify.py', env=env)
    assert loop.returncode != 0
    assert 'redirect 순환' in loop.stderr


def main():
    tests = [test_imports_fail_before_stale_body,
             test_direct_exec_preserves_args_and_uses_canonical_target,
             test_camera_force_flag_cannot_bypass_canonical_redirect,
             test_all_filesystem_hardware_entrypoints_are_guarded,
             test_non_redirect_tools_are_bound_to_reviewed_source,
             test_missing_target_and_self_loop_are_blocked]
    for test in tests:
        test()
        print(f'  PASS {test.__name__}')
    print(f'PASS — legacy authority {len(tests)}/{len(tests)}')


if __name__ == '__main__':
    main()
