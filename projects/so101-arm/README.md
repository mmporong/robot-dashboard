# SO-101 실기 제어 패널

실물 SO-101 팔로워를 브라우저에서 조작한다. 다른 프로젝트(`capstone-pick`·`slam`)의
대시보드는 **기록을 보는** 정적 페이지지만, 이쪽은 **팔을 움직이는** 라이브 페이지라
뒤에 서버가 붙는다.

```
브라우저 ──HTTP──> panel_server.py ──시리얼──> SO-101 (STS3215 ×6)
                        │        └──ctypes──> Orbbec Astra S (깊이)
                        └────────V4L2──────> 손목캠 (UVC)
```

## 실행

```bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot
export LD_LIBRARY_PATH=$HOME/AstraSDK/lib:$LD_LIBRARY_PATH   # 깊이 스트림용
python3 panel_server.py          # http://127.0.0.1:8765
```

시리얼 포트와 카메라 노드는 **자동 탐색**한다. USB 를 다시 꽂으면 `/dev/ttyACM*`·
`/dev/video*` 번호가 밀리는데(실측 3회), 고정 경로로 띄우면 서버가 조용히 죽은
경로를 붙들기 때문이다. 실행 중 끊겨도 읽기 실패가 이어지면 스스로 재연결한다.

## 구성

| 파일 | 역할 |
|---|---|
| `panel_server.py` | HTTP 서버 — `/state` 폴링 · `/cmd` 명령 · `/cam`·`/depth` MJPEG |
| `panel.html` | 화면. 카메라 2분할(손목·깊이)이 주인공, 조작은 우측 컬럼 |
| `tools/arm_gui.py` | 시리얼 전담 워커(`Worker`). 서버가 이걸 그대로 쓴다 |
| `tools/arm_lib.py` | 연결·보간 이동·rad↔deg 매핑. IK 는 캡스톤 `kinematics.py` 를 부른다 |
| `tools/astra.py` | Orbbec Astra S 깊이 — ctypes 바인딩 |
| `tools/scan_motors.py` | 버스 스캔(읽기 전용). 서보가 안 잡힐 때 1순위 도구 |
| `tools/servo_calib.py` | 손목캠 픽셀↔팔 이동량(px/m) 실측 |
| `tools/align_y.py` | 좌우 정렬 폐루프 |
| `tools/sweep_x.py` | 전후 스윕 (면적 최대점 탐색) |
| `tools/probe_floor.py` | 접촉으로 책상면 높이 등록 |
| `tools/pick_red.py` | 빨간 물체 파지 시퀀스 |
| `tools/servo_gain.json` | **실측 상수 저장소** — 아래 참조 |
| `tools/mapping.json` | 관절 부호·오프셋·홈 자세 |

## 실측 상수 (2026-08-18)

`tools/servo_gain.json` 에 있고, 전부 이 기체·이 카메라 배치에서만 유효하다.

| 값 | 의미 |
|---|---|
| `floor_z_m = -0.1037` | pan 축 기준 책상면 높이. 하강 목표를 계산으로 얻는다 |
| `y_to_px = -4578` | 좌우 1m 이동당 블롭 픽셀 변화. 정렬 루프의 이득 |
| `jaw_offset_z_m = -0.05` | 카메라가 물체를 크게 담는 자세에서도 죠는 이만큼 위에 있다 |
| `taught_grasp` | 사람이 직접 잡아 보여 준 파지 자세 |

**카메라를 옮기면 손목캠 관련 값(`y_to_px`·`jaw_offset_z_m`·기준선 306px)을 다시 재야
한다.** 마운트 위치에만 의존하는 값이다.

## 안전

- 속도는 **`Goal_Velocity` 가 `Maximum_Velocity_Limit`(공장 기본 65) 이하일 때만** 반영된다.
  초과하면 조용히 무시되고 최대 속도로 튄다 — 연결 시 상한을 254 로 올려 1~254 를
  1 unit ≈ 0.087°/s 로 쓴다(실측: vel 120→10.4°/s · 200→17.0°/s).
- 토크는 **한 서보씩 순차 인가**한다. 6개 동시 돌입 전류로 전원이 주저앉아 보드가
  USB 에서 떨어진 적이 있다.
- `⏹ 정지`는 감속 정지(여유 ≈7°, 자세 유지), `토크 OFF` 는 즉시 힘 제거(팔 처짐).
  스페이스바가 정지다.
- **`wrist_roll` 을 크게 돌릴 때는 회전 방향을 먼저 확인한다.** 같은 자세라도 +방향과
  -방향의 경유 경로가 달라, 한쪽은 손목캠이 구조물에 눌린다(2026-08-18 위험 상황).

## 알려진 한계

- **전후(x)는 손목캠으로 폐루프가 안 된다.** 팔이 x 로 움직여도 손목 자세가 거의
  안 변해 시야가 반응하지 않는다(실측 +8 px/m, 좌우는 -4578). 면적으로 대체하려
  했으나 거리와 단조 관계가 아니어서 두 번 실패했다 — 깊이 카메라로 푸는 것이 맞다.
- 흰 책상은 ToF 반사가 약해 깊이 유효율이 60% 대다.
