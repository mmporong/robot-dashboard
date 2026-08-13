"""대시보드에 넣을 카메라 프레임을 캡처한다 — 전방·손목 두 대를 같은 자세에서.

저장된 옛 프레임을 재활용하지 않는 이유: 그때의 무대·팔 자세를 알 수 없어
"이 관절값일 때 카메라가 이걸 봤다"는 대응이 깨진다. 대시보드는 그 대응으로
관절과 영상을 함께 스크럽하므로, 자세와 프레임을 **같이** 기록해야 한다.

사용 (ROS 환경 source 후): python3 grab_frames.py
출력: frames.json  {poses:[{name, q[5], front_b64, wrist_b64}], ...}
"""
import base64
import json
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
       'arm_wrist_flex', 'arm_wrist_roll']

# 실제 파이프라인이 지나는 자세들 — 대시보드 단계와 대응시킨다
POSES = [
    ('접힘',      [0.0,  -0.4,  1.0,  0.2,  0.0]),
    ('프리그래스프', [0.0,   0.62, 0.30, 0.66, 0.0]),
    ('파지',      [0.0,   1.15, 0.15, 0.28, 0.0]),
    ('들기',      [0.0,   0.15, 0.15, 1.28, 0.0]),
    ('투입 선회',   [-0.87, 0.30, 0.35, 1.00, 0.0]),
]
JPEG_Q = 62          # 용량 대 화질 — 640x480에서 이 정도면 20~30KB
SCALE = 0.55         # 패널 표시 크기에 맞춰 축소


class Grab(Node):
    def __init__(self):
        super().__init__('frame_grabber')
        self.br = CvBridge()
        self.front = None
        self.wrist = None
        self.create_subscription(Image, '/rgbd_camera/image', self._f, 1)
        self.create_subscription(Image, '/wrist_camera/image_raw', self._w, 1)
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

    def _f(self, m):
        self.front = self.br.imgmsg_to_cv2(m, 'bgr8')

    def _w(self, m):
        self.wrist = self.br.imgmsg_to_cv2(m, 'bgr8')

    def move(self, q, sec=2.5):
        t = JointTrajectory()
        t.joint_names = ARM
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q]
        p.time_from_start.sec = int(sec)
        p.time_from_start.nanosec = int((sec % 1) * 1e9)
        t.points = [p]
        self.pub.publish(t)

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)


def enc(img):
    if img is None:
        return None
    h, w = img.shape[:2]
    small = cv2.resize(img, (int(w * SCALE), int(h * SCALE)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
    return base64.b64encode(buf).decode() if ok else None


def main():
    rclpy.init()
    n = Grab()
    n.spin(2.0)
    if n.front is None and n.wrist is None:
        print('카메라 프레임이 오지 않는다 — 시뮬이 떠 있는지 확인하라')
        n.destroy_node(); rclpy.shutdown(); return 1

    out = []
    for name, q in POSES:
        n.move(q, 2.5)
        n.spin(3.2)                       # 도달 + 프레임 갱신 대기
        n.front = n.wrist = None
        n.spin(1.2)                       # 새 프레임만 받는다
        f, w = enc(n.front), enc(n.wrist)
        out.append({'name': name, 'q': [round(v, 4) for v in q],
                    'front': f, 'wrist': w})
        fb = len(f) if f else 0
        wb = len(w) if w else 0
        print(f'  {name:12s} front {fb // 1024:3d}KB  wrist {wb // 1024:3d}KB')

    n.move([0.0, -0.4, 1.0, 0.2, 0.0], 3.0)
    n.spin(3.0)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'frames.json')
    json.dump(out, open(path, 'w'), separators=(',', ':'))
    print(f'\n저장 {path}  {os.path.getsize(path) // 1024}KB')
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
