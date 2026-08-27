#!/usr/bin/env python3
"""차량 대시보드의 depth-free 계약과 현황 집계를 오프라인 검증한다."""
import csv
import json
import pathlib
import sys
import tempfile

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
import project as P                                           # noqa: E402
import read_runs as R                                         # noqa: E402


def test_vehicle_channels():
    active = [k for k, v in P.CHANNELS.items() if not v.get('legacy')]
    legacy = [k for k, v in P.CHANNELS.items() if v.get('legacy')]
    assert active == ['wrist', 'sim', 'screen'], active
    assert legacy == ['rgb', 'depth'], legacy
    assert '/*__STATUS__*/' in P.DATA


def test_status_evidence():
    old_yolo, old_pick = P.YOLO_LOG, P.PICK_LOG
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        P.YOLO_LOG = root / 'yolo.jsonl'
        P.PICK_LOG = root / 'pick.csv'
        P.YOLO_LOG.write_text('\n'.join(json.dumps({'target': t}) for t in
                                         ({'confidence': .7}, None, {'confidence': .8})))
        with P.PICK_LOG.open('w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=['ts', 'repo', 'result', 'reason'])
            w.writeheader()
            w.writerow({'ts': 't1', 'repo': 'bench', 'result': 'success', 'reason': ''})
            w.writerow({'ts': 't2', 'repo': 'so101_car', 'result': 'fail', 'reason': 'gate'})
        status = R.read_status()
    P.YOLO_LOG, P.PICK_LOG = old_yolo, old_pick
    assert status['yolo'] == {'frames': 3, 'hits': 2, 'rate_pct': 66.7,
                              'confidence': 0.40, 'gate': 'blocked'}
    assert status['pick']['cycles'] == 1 and status['pick']['success'] == 0
    assert status['pick']['last_reason'] == 'gate'


def test_live_panel_contract():
    page = (HERE / 'panel.html').read_text()
    server = (HERE / 'panel_server.py').read_text()
    assert 'src="/depth"' not in page and 'src="/rgb"' not in page
    assert 'value="so101_car"' in page
    assert '팬 잠금 없음' in page and '상태 끊김' in page
    assert 'class Depth(' not in server
    assert "self.path == '/blob'" not in server
    assert "self.path == '/depth'" not in server
    assert "self.path == '/rgb'" not in server
    assert "depth=False, pointmap=False" in server
    assert "'mirror_piece'" in server
    assert 'new AbortController()' in page
    assert "#joints input[type=range]" in page
    assert 'performance.now()-lastStateAt>1000' in page


def test_record_dashboard_contract():
    page = (HERE / 'dashboard.tpl.html').read_text()
    assert 'const STATUS = /*__STATUS__*/{};' in page
    assert 'SO-101 Mobile Manipulation' in page
    assert "filter(j=>j!=='shoulder_pan'" in page
    assert "const j0 =" not in page


def main():
    tests = [test_vehicle_channels, test_status_evidence,
             test_live_panel_contract, test_record_dashboard_contract]
    for test in tests:
        test()
        print(f'PASS — {test.__name__}')
    print(f'통과 — 차량 대시보드 {len(tests)}항목')


if __name__ == '__main__':
    main()
