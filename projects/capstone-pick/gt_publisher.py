"""Gazebo 실좌표를 ROS 토픽으로 내보낸다 — `/gt/cube` `/gt/robot` `/gt/trash`.

**왜 필요한가.** 이 프로젝트의 판정은 로그가 아니라 실좌표로 한다. 그런데 그 실좌표를
지금까지는 추적 스크립트가 `gz model -m <이름> -p` **CLI를 호출해** 받아 적었다.
그래서 `ros2 bag record`만으로 만든 bag에는 판정 근거가 통째로 없었다(실측: `live1.mcap`
163MB를 읽어도 "판정 불가 — 실좌표가 없다"로 끝났다). 이 노드가 그 구멍을 메운다.

## 왜 `ros_gz_bridge`를 안 쓰나

`/world/<월드>/pose/info`(`gz.msgs.Pose_V`)에 모든 모델 자세가 실려 있어서 처음엔 그것을
`tf2_msgs/TFMessage`로 브리지했다. **모델 이름이 버려진다** — 브리지가 넘겨준 메시지는
`frame_id`·`child_frame_id`가 전부 빈 문자열이라 어느 자세가 어느 모델인지 알 수 없다
(`tf` 토픽은 gz가 이름을 헤더에 넣어 줘서 되지만 `pose/info`는 아니다).

그래서 `gz topic -e --json-output`을 **스트림으로 읽는다.** 거기엔 이름이 그대로 있다.
줄마다 완전한 JSON 한 건이라 줄 단위로 파싱하면 된다.

`gz model -m <이름> -p`를 주기적으로 부르는 방법도 있지만 **한 번에 0.55초**가 걸린다
(실측). 모델 셋이면 초당 한 번도 못 돈다. 스트림은 58Hz로 들어온다.

사용 (`record_mcap.sh`가 대신 띄운다):
    python3 gt_publisher.py --cube pick_object_green
"""
import argparse
import json
import subprocess
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

# 월드 좌표라고 적어 둔다. 이 값을 base 기준으로 착각하면 통이 엉뚱한 데 그려진다.
FRAME = 'world'


class GroundTruth(Node):
    def __init__(self, models, rate_hz, world):
        super().__init__('gt_publisher')
        self.models = models                      # {모델 이름: 내보낼 토픽 이름}
        self.period = 1.0 / rate_hz
        self.last = {}
        self.pubs = {n: self.create_publisher(PoseStamped, f'/gt/{k}', 10)
                     for n, k in models.items()}
        self.proc = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', f'/world/{world}/pose/info', '--json-output'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._pump, daemon=True).start()
        self.get_logger().info(f'/world/{world}/pose/info → '
                               + ' · '.join(f'/gt/{k}' for k in models.values())
                               + f' ({rate_hz}Hz)')

    def _pump(self):
        for line in self.proc.stdout:
            line = line.strip()
            if not line.startswith('{'):
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue                # 스트림이 잘린 줄은 버린다. 다음 줄이 곧 온다
            self._emit(msg)

    def _emit(self, msg):
        # 시뮬이 58Hz로 전 모델을 흘리므로 그대로 내보내면 bag이 부푼다.
        # 판정은 물체가 어디 있느냐라 20Hz면 충분하다.
        now = self.get_clock().now()
        secs = now.nanoseconds / 1e9
        for pose in msg.get('pose', []):
            name = pose.get('name')
            if name not in self.models or secs - self.last.get(name, -1e9) < self.period:
                continue
            self.last[name] = secs
            # gz JSON은 **0인 필드를 통째로 생략한다.** `{"x": 0.15, "y": 0.62}`처럼
            # z가 없는 것이 정상이라 기본값을 0으로 두고 꺼내야 한다.
            pos = pose.get('position', {})
            rot = pose.get('orientation', {})
            p = PoseStamped()
            p.header.stamp = now.to_msg()
            p.header.frame_id = FRAME
            p.pose.position.x = float(pos.get('x', 0.0))
            p.pose.position.y = float(pos.get('y', 0.0))
            p.pose.position.z = float(pos.get('z', 0.0))
            p.pose.orientation.x = float(rot.get('x', 0.0))
            p.pose.orientation.y = float(rot.get('y', 0.0))
            p.pose.orientation.z = float(rot.get('z', 0.0))
            p.pose.orientation.w = float(rot.get('w', 1.0))
            self.pubs[name].publish(p)

    def destroy_node(self):
        self.proc.terminate()
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cube', default='pick_object_green',
                    help='집을 물체의 모델 이름. 색이 바뀌면 여기도 바뀐다')
    ap.add_argument('--robot', default='jdamr_cube')
    ap.add_argument('--trash', default='trash_can')
    ap.add_argument('--world', default='room')
    ap.add_argument('--rate', type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    node = GroundTruth({args.cube: 'cube', args.robot: 'robot', args.trash: 'trash'},
                       args.rate, args.world)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
