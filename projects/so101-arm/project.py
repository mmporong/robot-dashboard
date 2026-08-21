"""SO-101 실기 벤치 — 기록 대시보드 설정 (2026-08-21 신설).

같은 디렉터리의 `panel.html`/`panel_server.py` 는 **라이브** 화면(지금 팔을
움직인다)이고, 이쪽은 **기록** 화면이다(끝난 시행을 되돌려 본다). 데이터가
서로 다르다: 라이브는 서보에서 직접, 기록은 `~/so101_tools/media/` 에 남은
영상·상태 CSV 와 `~/so101_datasets/` 의 LeRobot 데이터셋에서 온다.

캡스톤(capstone-pick)과 다른 점: 저기는 Gazebo MCAP 한 파일에 관측·영상·실좌표가
같이 들어 있지만, 실기는 그런 것이 없다. 시행 하나는 `run_demo.sh` 가 남긴
**파일 묶음**(demo_<시각>_{wrist,rgb,depth,screen,sim}.mp4 + _state.csv)이고,
그 묶음을 시각으로 엮는 것이 read_runs.py 가 하는 일이다.
"""
import pathlib

REQUIRED = ['ARM_ORDER', 'GRIPPER', 'CHANNELS', 'MEDIA_ROOT', 'DATASET_ROOT',
            'TEMPLATE', 'DATA', 'VIDEO_BUDGET_MB']

# ── 기록에서 읽을 것 ────────────────────────────────────────────────
ARM_ORDER = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
             'wrist_flex', 'wrist_roll']
GRIPPER = 'gripper'

MEDIA_ROOT = pathlib.Path('~/so101_tools/media').expanduser()
DATASET_ROOT = pathlib.Path('~/so101_datasets').expanduser()

# 파일 접미사 → 화면에서 부를 이름·설명. run_demo.sh 의 6채널 규약과 맞춘다.
# `_realtime` 접미사가 붙은 판이 있으면 그쪽을 쓴다 — 무접미 판은 mpjpeg 를
# 벽시계 타임스탬프 없이 받아 2.5배속으로 굳은 기록이다(2026-08-20 실측).
CHANNELS = {
    'rgb':    {'label': '정면 (뎁스캠 컬러)', 'realtime': True},
    'wrist':  {'label': '손목캠',             'realtime': True},
    'depth':  {'label': '깊이',               'realtime': True},
    'sim':    {'label': 'MuJoCo 미러',        'realtime': False},
    'screen': {'label': '패널 화면',          'realtime': False},
}
# 화면에 세울 채널 순서 (없는 채널은 그냥 빠진다)
CHANNEL_ORDER = ['rgb', 'wrist', 'sim', 'depth', 'screen']

# ── 판정 ───────────────────────────────────────────────────────────
# 실기에는 Gazebo 실좌표 같은 심판이 없다. 로그의 "PICK_SUCCESS" 를 그대로 믿지
# 않는 것이 이 벤치의 규칙이라(2026-08-19 실측: 로그는 성공, 실물은 헛집음),
# 자동 판정 대신 **계측된 사실만** 싣는다: 그리퍼가 닫힌 각도, 최고 온도,
# 토크가 꺼진 순간. 성공/실패 라벨은 사람이 영상을 보고 붙인다.
GRIP_CLOSED_DEG = 25.0      # 이보다 닫히면 무언가를 물었거나 빈 죠가 다물렸다
TEMP_WARN_C = 55.0          # 이 위는 화면에 경고색

# ── 화면 ───────────────────────────────────────────────────────────
TEMPLATE = 'dashboard.tpl.html'
DATA = {
    '/*__RUNS__*/': 'runs.json',
    '/*__DATASETS__*/': 'datasets.json',
}
TREND = False               # 판(epoch) 누적 회귀는 아직 쓰지 않는다

# 영상은 다시 인코딩해 예산에 맞춘다. 원본은 시행 하나가 60MB를 넘기도 해서
# (demo_160236 5채널 합 73MB) 그대로 실을 수 없다.
# 영상을 문서에 묻지 않고 **옆 파일로 뺀다.** 이 화면은 아티팩트로 발행하는 것이 아니라
# 로컬 서버로 보므로 단일 파일일 이유가 없다. 빼면 base64의 33% 손해가 없어지고,
# 예산에 눌려 화질을 깎을 일도 없어지며, 브라우저가 보는 채널만 받아 온다.
VIDEO_DIR = 'media'
VIDEO_BUDGET_MB = 11.0      # VIDEO_DIR 를 쓰면 안 쓰인다 (묻어 넣을 때만 필요)
VIDEO_FPS = 4
VIDEO_CRF = 24              # 파일로 빼므로 예산에 쫓기지 않는다
VIDEO_MAX_W = 480           # 기본 폭
# **채널마다 필요한 화질이 다르다.** 확대해서 들여다보는 것은 정면·손목캠이고,
# 화면 녹화나 시뮬 미러는 배경으로만 둔다. 예산은 하나뿐이니 필요한 쪽에 몰아준다.
# 값은 기준 crf에 더하는 차 — 클수록 나빠지고 작아진다
# (실측 2026-08-21: 같은 원본이 crf 28에 5.9MB, crf 40에 2.1MB).
VIDEO_CRF_BY_CHANNEL = {'rgb': -4, 'wrist': -4, 'sim': +4, 'depth': +2, 'screen': +8}
# 폭도 채널마다. 손목캠은 원본이 작아(352) 그대로 두고, 배경 채널은 더 줄인다.
VIDEO_W_BY_CHANNEL = {'rgb': 640, 'wrist': 512, 'depth': 480, 'sim': 400, 'screen': 400}
VIDEO_PRESET = 'veryfast'   # 시행 하나가 채널 다섯이라 slow 는 빌드가 분 단위로 는다
# 한 화면에 세울 시행 수 — 최신부터. 전부 실으면 아티팩트 한도를 넘는다.
MAX_RUNS = 8
# **영상까지** 실을 시행. 나머지는 계측·시계열만 실린다(용량은 영상이 전부다).
# 비워 두면 가장 최근 시행 하나를 자동으로 고른다.
# 너무 짧은 시행은 걸러낸다. 기록이 끊겼거나(0초) 팔이 채 펴지기도 전에 멈춘 것들이라
# 화면에 세워 봐야 읽을 것이 없고, 칩만 늘어나 정작 볼 시행을 가린다.
# 실측(2026-08-21): 9시행 중 demo_163812 하나가 0.0초·표본 0이었다.
MIN_DUR_S = 10.0

SHOW = ['2026-08-20/demo_160236']
# 시계열은 이 간격으로 솎는다 [초]. CSV 는 20Hz라 2700행짜리도 있다.
SERIES_STRIDE_S = 0.5
