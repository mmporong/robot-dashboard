# SO-101 Mobile Manipulation 대시보드

차량에 장착한 SO-101의 라이브 제어와 실기 기록 검수를 한 프로젝트 화면에서 다룬다.
제어·캘리브레이션·MuJoCo·LeRobot 코드는 `~/so101-mobile-manipulation`이 정본이며,
이 디렉터리는 브라우저 UI와 기록 대시보드만 소유한다.

## 현재 구성

```text
손목캠 ───────────────┐
SO-101 서보 ──────────┼─ 노트북: 패널·YOLO·ACT·Nav2
Pi /scan·/odom·배터리 ┘               │
                                      └─ Collision Monitor → Pi /cmd_vel
```

- 비전은 손목캠 단독이다. Astra depth 카메라는 차량 프로필에서 사용하지 않는다.
- 라이브 패널은 손목캠과 차량 받침대가 포함된 MuJoCo 미러를 2분할로 보여 준다.
- 데이터셋은 관절 상태·액션·손목캠만 기록하며 기본 repo ID는 `so101_car`다.
- 과거 벤치의 rgb/depth 영상은 기록 되감기 호환용 legacy 채널로만 읽는다.
- 최신 하드웨어 제약과 교시값은 `~/so101-mobile-manipulation/HANDOFF_CAR.md`가 정본이다.

## 실행

```bash
cd "$HOME/robot-dashboard/projects/so101-arm"
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate lerobot
python3 ./panel_server.py
```

브라우저 주소는 `http://127.0.0.1:8765`다. 서버는 외장 UVC 손목캠과 검증된
SO-101 시리얼 어댑터를 자동 탐색한다. 팔이 꺼져 있어도 기록·MuJoCo 검수용으로
패널 자체는 기동한다.

기록 대시보드는 저장소 루트에서 갱신한다.

```bash
cd "$HOME/robot-dashboard"
"$HOME/miniforge3/envs/lerobot/bin/python" ./dash.py so101-arm all
"$HOME/miniforge3/envs/lerobot/bin/python" ./dash.py so101-arm check
```

`check`는 설정뿐 아니라 `dashboard.html`이 템플릿과 JSON 입력보다 최신인지도 확인한다.

## 화면에서 확인할 안전 상태

- 연결·캘리브레이션·토크
- 팬 잠금 중심과 허용폭: 현재 기준 `−15.6° ±7.0°`
- 최고 서보 온도와 최저 전압
- 상태 수신 실패 시 이동 버튼 자동 비활성화
- YOLO 케이블 가림 재검증 게이트

`정지`와 `토크 OFF`는 상태가 끊겨도 계속 누를 수 있다. 조그·IK·홈 이동은 연결,
캘리브레이션, 토크가 모두 확인된 때만 활성화된다.

## 파일 역할

| 파일 | 역할 |
|---|---|
| `panel_server.py` | 라이브 HTTP 서버. 정본의 `arm_gui.py`·`ds_record.py`를 import |
| `panel.html` | 손목캠·MuJoCo·안전 HUD·제어·기록 화면 |
| `project.py` | 기록 채널, 차량 현황 원자료, 영상 빌드 설정 |
| `read_runs.py` | 실기 영상·CSV·LeRobot·YOLO·pick 로그를 JSON으로 변환 |
| `dashboard.tpl.html` | 기록 되감기와 차량 현황 화면 |
| `depth_daemon.py` | 과거 Astra 벤치 재현용 legacy 도구. 차량 패널은 실행하지 않음 |

## 현재 YOLO 게이트

2026-08-27 정지 관측에서 노트북의 YOLO 단독 추론은 평균 5.3ms, P95 7.1ms였다.
케이블이 큐브를 가린 상태의 `confidence=0.40` 검출률은 11/100이었다. 케이블을
정리한 뒤 양성 100프레임 검출률 95% 이상, 음성 100프레임 오검출 0건을 모두
통과하기 전에는 YOLO 출력을 비영점 주행에 연결하지 않는다.
