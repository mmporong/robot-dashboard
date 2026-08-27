#!/usr/bin/env python3
import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from _canonical_redirect import redirect_if_main as _redirect
_redirect(__name__, 'sim/mirror_daemon.py')
