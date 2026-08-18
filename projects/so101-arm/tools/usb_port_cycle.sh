#!/bin/sh
# Astra 뎁스캠이 붙은 USB 포트의 전원(VBUS)만 껐다 켠다.
#
# 왜 필요한가: Astra 는 열거 도중 멈추는 일이 있다(port state 가 configured 에
# 이르지 못하고 default/addressed 에 고착). 이 상태는 USBDEVFS_RESET·드라이버
# unbind/bind·authorized 토글 어느 것으로도 안 풀리고, 장치 전원이 실제로 끊겨야
# 초기화된다. 원격 작업 중이면 케이블을 뽑을 사람이 없으므로 포트 전원을 쓴다.
#
# 대상 포트를 파일에 박아 둔 이유: 인자로 경로를 받으면 이 스크립트에 무암호
# sudo 를 주는 순간 임의의 sysfs 노드에 쓸 수 있게 된다. 포트가 바뀌면 이 파일을
# 고칠 것(그러려면 root 권한이 필요하고, 그게 의도한 바다).
#
# 설치 (사용자 쓰기 불가여야 무암호 sudo 가 안전하다):
#   sudo install -o root -g root -m 755 usb_port_cycle.sh /usr/local/sbin/
#   echo "$USER ALL=(root) NOPASSWD: /usr/local/sbin/usb_port_cycle.sh" \
#     | sudo tee /etc/sudoers.d/astra-usb
#   sudo chmod 440 /etc/sudoers.d/astra-usb
#
# 사용: sudo /usr/local/sbin/usb_port_cycle.sh [전원차단초, 기본 8]

set -eu

PORT=/sys/bus/usb/devices/usb1/1-0:1.0/usb1-port1/disable

# 대기 시간은 숫자만 받는다 — 무암호 sudo 로 도는 스크립트라 인자를 신뢰하지 않는다.
SECS=${1:-8}
case "$SECS" in
    ''|*[!0-9]*) echo "대기 시간은 정수여야 합니다: $SECS" >&2; exit 2 ;;
esac
[ "$SECS" -le 60 ] || { echo "대기 시간 상한은 60초입니다" >&2; exit 2; }

[ -w "$PORT" ] || { echo "포트 제어 노드에 쓸 수 없습니다: $PORT" >&2; exit 1; }

echo 1 > "$PORT"
echo "포트 전원 차단 — ${SECS}초 대기"
sleep "$SECS"
echo 0 > "$PORT"
echo "포트 전원 복구"
