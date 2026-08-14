#!/bin/bash
# 시행 N회를 `ros2 bag record`로 MCAP에 직접 담는다 — 변환기를 거치지 않는 경로.
#
# `to_mcap.py`는 옛 JSONL을 옮기는 레거시 변환기다. 여기서 나온 파일은 `read_mcap.py`가
# 변환본과 **구별 없이** 읽어야 한다 — 그게 표준 포맷을 쓰는 이유이자 리더가 제대로
# 됐는지 보는 시험이다.
#
# **실좌표를 같이 담는다.** 예전엔 추적 스크립트가 `gz model -p` CLI로 긁어 JSONL에
# 적었기 때문에 bag만으로는 판정 근거가 없었다(실측: live1.mcap 163MB → "판정 불가").
# 지금은 `gt_publisher.py`가 `/world/room/pose/info`를 받아 `/gt/*`로 내보내고 그것을
# 함께 기록한다. 기록 한 번으로 판정까지 닫힌다.
#
# 카메라는 **원본 해상도(640×480) 압축 토픽**을 그대로 받는다. 옛 JSON 녹화기는 찍는
# 순간 0.34배로 줄여 저장해서 나중에 키울 수가 없었다(217×163이 그래서 나왔다).
#
# 감시 디렉터리 계약: 반쯤 쓰인 파일이 인박스에 보이면 안 된다. 기록은 staging에 하고
# 끝난 뒤 `mv`로 옮긴다(같은 파일시스템이라 원자적이다).
#
# `pkill -f`는 쓰지 않는다 — 패턴이 자기 셸 argv에 걸려 스스로를 죽인다. 오래 도는 것은
# 전부 systemd 유저 유닛으로 띄우고 유닛 이름으로 멈춘다.
#
# `set -u`는 쓸 수 없다 — ROS의 setup.bash가 미설정 변수를 참조해 그 자리에서 죽는다.
#
# 사용: record_mcap.sh [시행이름] [횟수] [세대]
#   예: record_mcap.sh bagA 3 arm-v3-bag
NAME="${1:-bagA}"
TRIALS="${2:-3}"
EPOCH="${3:-arm-v3-bag}"
CUBE="${CUBE_MODEL:-pick_object_green}"
STAGE="$HOME/capstone_tools/bag_staging"
INBOX="$HOME/capstone_tools/mcap"
LOGS="$HOME/capstone_tools/logs"
HERE="$(cd "$(dirname "$0")" && pwd)"
TOPICS=(/joint_states /tf /tf_static /odom
        /gt/cube /gt/robot /gt/trash
        /rgbd_camera/image/compressed /wrist_camera/image_raw/compressed)

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"

rm -rf "$STAGE"; mkdir -p "$STAGE" "$INBOX"

# 추적 JSONL은 append다. 그냥 돌리면 보존하기로 한 옛 기록에 새 시행이 섞인다.
KEEP="$LOGS/armonly_trace.jsonl"
[ -f "$KEEP" ] && mv "$KEEP" "$KEEP.aside"
restore() { [ -f "$KEEP.aside" ] && mv -f "$KEEP.aside" "$KEEP"; }
trap restore EXIT

unit() {   # unit <이름> <명령>
  systemctl --user stop "$1" 2>/dev/null
  systemctl --user reset-failed "$1" 2>/dev/null
  systemd-run --user --collect \
    --setenv=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST --setenv=GZ_PARTITION=lim-capstone \
    --unit="$1" bash -c "source /opt/ros/jazzy/setup.bash && \
      source \$HOME/jdamr_cube_ws/install/setup.bash && $2"
}

# Gazebo 모델 자세를 /gt/* 로. `ros_gz_bridge`는 쓰지 않는다 — Pose_V → TFMessage 변환이
# **모델 이름을 버려서**(frame_id가 빈 문자열) 어느 자세가 어느 모델인지 알 수 없다.
unit capstone-gt "exec python3 '$HERE/gt_publisher.py' --cube '$CUBE'"
sleep 6
# 실좌표가 실제로 흐르는지 확인하고 간다 — 없으면 판정 불가짜리 bag이 또 나온다
if ! timeout 15 ros2 topic echo /gt/cube --once >/dev/null 2>&1; then
  echo "실좌표(/gt/cube)가 안 나온다 — 브리지나 모델 이름($CUBE)을 확인할 것"
  systemctl --user stop capstone-gt 2>/dev/null
  exit 1
fi
echo "실좌표 확인됨"

# **시행 하나가 파일 하나다.** 한 bag에 여러 회차를 몰아 넣으면 시행 경계가 기록에
# 안 남는다(옛 JSONL은 `trial` 필드로 갈랐지만 bag에는 그런 것이 없다). 그래서 회차마다
# 기록을 새로 열고 닫는다.
for i in $(seq 1 "$TRIALS"); do
  run="${NAME}${i}"
  echo
  echo "── 시행 $i/$TRIALS · $run ──"
  unit capstone-bag "exec ros2 bag record -s mcap -o '$STAGE/$run' ${TOPICS[*]}"
  sleep 5

  python3 "$HOME/jdamr_cube_ws/src/jdamr_cube_ros/capstone_pick/tools/armonly_pick.py" 1 0
  rc=$?

  # SIGINT라야 rosbag2가 파일을 닫고 인덱스를 쓴다. TERM으로 끊으면 요약이 없는 파일이 남는다.
  systemctl --user kill --signal=SIGINT capstone-bag 2>/dev/null
  for _ in $(seq 1 30); do
    systemctl --user is-active --quiet capstone-bag || break
    sleep 1
  done
  systemctl --user stop capstone-bag 2>/dev/null

  src=$(find "$STAGE/$run" -name '*.mcap' | head -1)
  if [ -z "$src" ]; then echo "  MCAP이 안 나왔다 — 건너뜀"; continue; fi
  mv "$src" "$INBOX/$run.mcap"
  # bag 안에는 우리 메타데이터를 넣을 자리가 없다. 옆에 떨어뜨리면 리더가 읽는다.
  cat > "$INBOX/$run.run.json" <<JSON
{"kind": "팔 단독", "epoch": "$EPOCH", "trash_source": "gz", "clock": "sim",
 "source": "ros2 bag record", "cube_model": "$CUBE", "trial": "$i"}
JSON
  [ -f "$LOGS/armonly_trace.jsonl" ] && mv "$LOGS/armonly_trace.jsonl" "$LOGS/${run}_trace.jsonl"
  echo "  인박스에 놓음: $run.mcap ($(du -h "$INBOX/$run.mcap" | cut -f1)) · 종료코드 $rc"
done

systemctl --user stop capstone-gt 2>/dev/null
echo
echo "끝 — $(ls "$INBOX/${NAME}"*.mcap 2>/dev/null | wc -l)개 기록"
