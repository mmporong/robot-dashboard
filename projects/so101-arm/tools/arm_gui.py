#!/usr/bin/env python3
"""Legacy path shim. 실행은 canonical arm_gui.py로만 넘긴다."""
from _canonical_redirect import redirect_if_main

redirect_if_main(__name__, 'arm_gui.py')
raise RuntimeError(
    'legacy arm_gui import 차단 — SO101_CANONICAL_DIR의 arm_gui를 import하세요')
