"""레거시 redirect wrapper의 실행 전 prefix 계약."""
import hashlib
import pathlib


CANONICAL_REDIRECT_HELPER_SHA256 = (
    'c9787a61f010a3723d29e44c4f556870b219d92d5b9773a3069227fa14f5c0af'
)

# Redirect/차단 대상이 아닌 도구도 이름만으로 예외 처리하지 않는다. 이 목록은
# 검토된 소스 자체를 승인하므로 새 direct-exec/import 부작용이 들어오면 실패한다.
APPROVED_STATIC_TOOL_SHA256 = {
    'ds_record.py': '63c9fca0877f20594c5940b95b95dd737447d7ab93cec49e92bbfa5fd338a70f',
    'log_state.py': '723696bf540cef32fc511be058acbf508fe77df55ed0178dc0a68dce11f04b43',
    'sim/frame_fit.py': '299d4c142ef2c81611c54da3f30eed5094464cb5e99926baf017f4baa7cedc9c',
    'sim/gen_ref_poses.py': '1536861571943d4ff09aef77f5fbdb7d2d7fcd4696a1a4608a55f5afd31461b3',
    'sim/sim_core.py': '65dace76298f123b46a5b9fe31f3686d84fae79c351e7d927a2eef44a707ab86',
    'sim/sim_view.py': '2957e98a8120eef5f0925b8d43f504d635ffacb36fcd0a7376c813b7fd553f4b',
}
APPROVED_OFFLINE_TEST_SHA256 = {
    'sim/test_sim_mirror.py': '9b9e6de736ba464eddfce4ab7aa1874fa1ca7dda891b82f8d18b2662ee10d039',
    'test_c1_control.py': '9e79ccc4e8335bbbe2ac643ff29b7f38a1afa2afaf062c561d8fc47724b99054',
    'test_drop_to_box.py': '113a311955e8650217e893f5e83c59274b5ce337d3480ed74f782f08237e9936',
    'test_ds_record.py': 'b2269ab9554ccfca168dbd458a93cdeac472845c8fd1c0fb55d261f086f47db9',
    'test_e2e_handeye.py': '1975e84dec5e47ae08b1b1a95b8f5d728d33863bff51f04ecaffd6aa113a600e',
    'test_park_path.py': 'f2210ed303f85386983b64540a6450519f8694b7c16d0a180dbb30cf00e734d2',
    'test_pick_demo.py': '82eb3830185bdcc193ea7d2b2f863631d4a4bbaaaa1dd9bcbcdb6bf4b57c2b0e',
    'test_place_down.py': '3bcd0fae1582bd4383bd684c3eb161ff5245cf9be9671f572f774ab16d3933dd',
    'test_stop_velocity.py': '976505f45f0b8458319cf73cf07f77914fe49e5bfe60a558f93ff77012cfcd31',
    'test_unfold_path.py': '3b628b40504bb411e9576036a79bd687114ccf5934e5cd1905a131c6697af169',
}


def expected_python_prefix(relative, target):
    relative = pathlib.PurePosixPath(relative)
    if relative.parent == pathlib.PurePosixPath('.'):
        return [
            '#!/usr/bin/env python3',
            'from _canonical_redirect import redirect_if_main as _redirect',
            f"_redirect(__name__, '{target}')",
        ]
    depth = len(relative.parent.parts)
    return [
        '#!/usr/bin/env python3',
        'import pathlib, sys',
        ('sys.path.insert(0, str(pathlib.Path(__file__).resolve()'
         f'.parents[{depth}]))'),
        'from _canonical_redirect import redirect_if_main as _redirect',
        f"_redirect(__name__, '{target}')",
    ]


def has_exact_python_prefix(source, relative, target):
    expected = expected_python_prefix(relative, target)
    return source.splitlines()[:len(expected)] == expected


def expected_shell_prefix(relative, target):
    relative = pathlib.PurePosixPath(relative)
    depth = len(relative.parent.parts)
    helper = '../' * depth + '_canonical_redirect.py'
    return [
        '#!/usr/bin/env bash',
        f'exec python3 "$(dirname "$0")/{helper}" {target} "$@"',
    ]


def has_exact_shell_prefix(source, relative, target):
    expected = expected_shell_prefix(relative, target)
    return source.splitlines()[:len(expected)] == expected


def is_approved_redirect_helper(source):
    return hashlib.sha256(source.encode()).hexdigest() == CANONICAL_REDIRECT_HELPER_SHA256


def is_approved_static_tool(source, relative):
    expected = APPROVED_STATIC_TOOL_SHA256.get(pathlib.PurePosixPath(relative).as_posix())
    return expected is not None and hashlib.sha256(source.encode()).hexdigest() == expected


def is_approved_offline_test(source, relative):
    expected = APPROVED_OFFLINE_TEST_SHA256.get(
        pathlib.PurePosixPath(relative).as_posix())
    return expected is not None and hashlib.sha256(source.encode()).hexdigest() == expected


def has_exact_arm_gui_prefix(source):
    return source.splitlines()[:5] == [
        '#!/usr/bin/env python3',
        '"""Legacy path shim. 실행은 canonical arm_gui.py로만 넘긴다."""',
        'from _canonical_redirect import redirect_if_main',
        '',
        "redirect_if_main(__name__, 'arm_gui.py')",
    ]


def is_exact_blocked_shell(source):
    return source.splitlines()[:3] == [
        '#!/bin/sh',
        'echo "legacy 실행 차단 — canonical usb_port_cycle.sh 진입점이 없습니다" >&2',
        'exit 2',
    ]
