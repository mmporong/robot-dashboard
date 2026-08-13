#!/bin/bash
# 시행 하나를 `ros2 bag record`로 MCAP에 직접 담는다 — 변환기를 거치지 않는 경로.
#
# `to_mcap.py`는 옛 JSONL을 옮기는 일회성 도구다. 앞으로의 기록은 여기서 나온다.
# 이 스크립트가 만든 파일은 `read_mcap.py`가 변환본과 **구별 없이** 읽어야 한다 —
# 그게 표준 포맷을 쓰는 이유이자, 리더가 제대로 됐는지 보는 시험이다.
#
# 감시 디렉터리 계약: 반쯤 쓰인 파일이 인박스에 보이면 안 된다. 기록은 staging에
# 하고, 다 끝난 뒤에 `mv`로 옮긴다(같은 파일시스템이라 원자적이다).
#
# `pkill -f` 는 쓰지 않는다 — 패턴이 자기 셸 argv에 걸려 스스로를 죽인다.
# 오래 도는 것은 전부 systemd 유저 유닛으로 띄우고 유닛 이름으로 멈춘다.
#
# 사용: ~/robot-dashboard-spec/record_mcap.sh [시행이름]   (기본 live1)
# `set -u`는 쓸 수 없다 — ROS의 setup.bash가 미설정 변수를 참조해 그 자리에서 죽는다
NAME="${1:-live1}"
STAGE="$HOME/capstone_tools/bag_staging"
INBOX="$HOME/capstone_tools/mcap"
LOGS="$HOME/capstone_tools/logs"
TOPICS=(/joint_states /tf /tf_static /odom
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

systemctl --user stop capstone-bag 2>/dev/null
systemctl --user reset-failed capstone-bag 2>/dev/null
systemd-run --user --collect \
  --setenv=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST --setenv=GZ_PARTITION=lim-capstone \
  --unit=capstone-bag bash -c \
  "source /opt/ros/jazzy/setup.bash && source \$HOME/jdamr_cube_ws/install/setup.bash && \
   exec ros2 bag record -s mcap -o '$STAGE/$NAME' ${TOPICS[*]}"
sleep 5
echo "기록 시작 — 토픽 ${#TOPICS[@]}개"

python3 "$HOME/jdamr_cube_ws/src/jdamr_cube_ros/capstone_pick/tools/armonly_pick.py" 1 0
rc=$?

# SIGINT라야 rosbag2가 파일을 닫고 인덱스를 쓴다. TERM으로 끊으면 요약이 없는
# 파일이 남아 `ros2 bag info`가 전체를 훑어야 한다.
systemctl --user kill --signal=SIGINT capstone-bag 2>/dev/null
for _ in $(seq 1 20); do
  systemctl --user is-active --quiet capstone-bag || break
  sleep 1
done
systemctl --user stop capstone-bag 2>/dev/null

src=$(find "$STAGE/$NAME" -name '*.mcap' | head -1)
if [ -z "$src" ]; then echo "MCAP이 안 나왔다 — $STAGE/$NAME 확인"; exit 1; fi
mv "$src" "$INBOX/$NAME.mcap"
[ -f "$LOGS/armonly_trace.jsonl" ] && mv "$LOGS/armonly_trace.jsonl" "$LOGS/${NAME}_trace.jsonl"
echo "인박스에 놓음: $INBOX/$NAME.mcap ($(du -h "$INBOX/$NAME.mcap" | cut -f1))  · 시행 종료코드 $rc"
