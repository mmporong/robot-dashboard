"""캡스톤 pick-and-place — 이 프로젝트에만 해당하는 값 전부.

`core/`는 이 파일 하나만 보고 돈다. 다른 로봇·다른 과제로 옮길 때 **고쳐야 하는 곳은
여기뿐**이고, 그게 사실인지는 `python3 dash.py <프로젝트> check`로 확인한다.

무대: SO-101 팔을 얹은 JDAMR 차체가 큐브를 집어 쓰레기통에 넣는다.
"""

# ── 기록에서 읽을 것 ────────────────────────────────────────────────
# 관절 이름은 `/joint_states`의 `name` 배열과 맞아야 한다. 순서가 아니라 **이름으로**
# 찾으므로, 발행자가 순서를 바꿔도 상관없다.
ARM_ORDER = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
             'arm_wrist_flex', 'arm_wrist_roll']
GRIPPER = 'arm_gripper'
# 손끝 위치를 얻을 TF 경로. `/tf`는 부모→자식 한 칸씩만 실리므로 체인을 거슬러 합성한다.
JAW_FRAME, BASE_FRAME = 'arm_gripper_frame_link', 'base_footprint'
# 카메라 토픽 → 화면에서 부를 이름. 원본(`/image`)과 압축(`/image/compressed`)을 모두 받는다.
CAMS = {'/rgbd_camera/image/compressed': 'front', '/rgbd_camera/image': 'front',
        '/wrist_camera/image_raw/compressed': 'wrist', '/wrist_camera/image_raw': 'wrist'}

# ── 판정 규칙 ──────────────────────────────────────────────────────
# 통 개구부는 136mm각(반폭 68mm), 벽 상단 0.090m. 물체 중심이 개구부 안이고 높이가
# 벽 상단보다 낮아야 통 안이다. **이 규칙이 이 프로젝트의 정체성**이라 core에 두지 않는다.
OPEN_HALF, WALL_TOP = 0.068, 0.090


def judge(cube, trash):
    """마지막 물체 좌표와 통 좌표로 성공 여부. None이면 판정 불가."""
    if cube is None or trash is None:
        return None
    dx, dy = cube[0] - trash[0], cube[1] - trash[1]
    return bool(abs(dx) < OPEN_HALF and abs(dy) < OPEN_HALF and cube[2] < WALL_TOP)


# ── 화면 ───────────────────────────────────────────────────────────
# 화면에 세울 시행. 실패만 모아 두면 "무엇이 정상인지"의 기준이 없어 읽을 수 없다.
# 성공은 팔 단독 쪽에만 있다 — 주행 76시행 중 실좌표로 통에 들어간 것은 하나도 없고,
# 그 사실 자체가 이 프로젝트의 현재 상태다.
# 2026-08-14 재녹화본으로 바꿨다. 옛 A1~A3은 녹화 시점에 0.34배로 줄여 저장한 것이라
# 217×163이 한계였고, 키울 방법이 없다. bagA*는 `ros2 bag record`가 640×480 압축 토픽을
# 그대로 받은 것이고 실좌표까지 같은 파일에 들어 있다. 성공 2·실패 1이 섞여 있는데
# 그게 더 낫다 — 성공만 세워 두면 무엇이 잘못된 모습인지 견줄 것이 없다.
SHOW = ['bagA1', 'bagA2', 'bagA3', 'drive_14', 'drive_10', 'drive_17', 'drive_13']
# 옛 화면이 쓰던 이름. 시행 번호를 바꾸면 링크를 공유한 사람의 화면이 달라진다.
LABEL = {'drive_14': 14, 'drive_10': 10, 'drive_17': 17, 'drive_13': 13}
TEMPLATE = 'dashboard.tpl.html'
# 템플릿의 플레이스홀더 → 채울 파일
DATA = {
    '/*__RUNS__*/': 'runs.json',
    '/*__CHAIN__*/': 'urdf_chain.json',
    '/*__FRAMES__*/': 'run_frames.json',
    '/*__TREND__*/': 'trend.json',
}
# 아티팩트 한 장에 다 들어가야 한다. 넘으면 솎되, 솎은 사실을 반드시 찍는다.
FRAME_BUDGET_MB = 4.2
# 원본 이미지가 실린 bag에서 프레임을 구울 때 쓴다(압축 토픽이면 그대로 쓴다).
# 압축 토픽을 받으므로 여기서 줄이지 않는다 — 640×480 그대로 들어온다.
JPEG = {'scale': 1.0, 'quality': 80}
# 카메라를 **비디오로** 싣는다. 낱장 JPEG은 같은 그림을 매번 통째로 실어서, 같은 용량에
# 해상도를 8.7배 키울 수 있는 자리를 버린다(실측 4.30MB → 1.72MB).
# `False`로 두면 옛 방식(낱장)으로 돌아간다 — 카메라가 없는 시행은 어느 쪽이든 빈다.
VIDEO = True
VIDEO_FPS = 4          # 되감기 반응과 용량의 절충. 키프레임은 1초마다 박는다
VIDEO_CRF = 26
# 비디오 예산은 낱장 예산과 따로 잡는다. 4.2MB는 **낱장 기준**으로 정한 값이고,
# 비디오는 같은 그림을 3배 가까이 접으므로 같은 숫자를 쓰면 화질을 공연히 버린다.
# 아티팩트 한도가 16MB라 10MB면 나머지(시행 기록·회귀·템플릿)에 여유가 남는다.
VIDEO_BUDGET_MB = 10.0

# ── 누적·회귀 ──────────────────────────────────────────────────────
# 기록되지 않은 시도. **분모 문제**다 — 무대 배치 단계에서 끊긴 시행은 추적 파일에
# 한 줄도 안 남아서, 기록만 세면 성공률이 실제보다 높게 나온다.
ATTEMPTS = {
    # 2026-08-13 캡스톤 세션 보고: 3회 중 1회가 "무대 배치 실패"로 끝나 추적이 없다.
    'arm-v2': 3,
}
EPOCH_NOTE = {
    'arm-v1': '월드에 큐브가 하나뿐이던 무대 — 방해 큐브가 없어 차체가 올라탈 일이 없었다',
    'arm-v2': '2026-08-13 무대 개편 — 방해 큐브를 벽 속에 버리던 것을 고치고 세 큐브를 팔이 닿는 자리에 모았다',
    'drive-v1': '이동·접근까지 포함 — 통 인식에 막혀 실좌표로 통에 들어간 것이 하나도 없다',
}

# ── LeRobot 내보내기 ───────────────────────────────────────────────
TASK = 'put the cube in the trash bin'
ROBOT_TYPE = 'so101_on_jdamr_cube'
CAVEAT = (
    'action = next observation.state (position-controlled arm); the source recording '
    'contains measured joint positions only, no commanded setpoints. '
    'next.success is ground truth from the Gazebo object pose, not a model estimate.'
)
