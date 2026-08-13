# robot-dashboard

ROS 2 실행 기록(MCAP)을 되감아 **로그가 아니라 실좌표로 다시 판정하는** 한 장짜리 관제 화면.

만든 계기는 하나다 — 캡스톤 pick-and-place에서 코드가 로그에 `PICK_SUCCESS`를 찍었는데
시뮬레이터에서 물체의 실제 좌표를 재 보니 통 밖에 떨어져 있었다. 주행 40시행 중
**4건이 그런 거짓 성공**이었고, 로그만 봐서는 알 수 없었다.

발행본: <https://claude.ai/code/artifact/7e10dd4c-b1ab-41f1-b0bd-6ce55f8c1067>

## 무엇을 하나

| | |
|---|---|
| **되감기** | 관절·카메라·실좌표를 한 시계 위에서 연속 재생. 3D 자세와 평면도가 같이 움직인다 |
| **재판정** | 파일에 실린 것은 실좌표뿐이다. 성공 여부는 **읽는 쪽에서 다시 센다** |
| **누적·회귀** | 시행이 쌓이면 무엇이 언제부터 나빠졌는지 찾는다. 판정에는 순열 검정으로 유의확률을 붙인다 |
| **내보내기** | 같은 기록을 LeRobotDataset v3.0으로. 학습 생태계에 붙는다 |

## 쓰는 법

```bash
python3 dash.py capstone-pick check     # 설정이 온전한지
python3 dash.py capstone-pick all       # read → trend → build
python3 dash.py capstone-pick probe ~/capstone_tools/mcap/live1.mcap
python3 dash.py capstone-pick selftest  # 회귀 탐지기 귀무모형 시험
```

읽는 쪽은 **ROS가 없어도 돈다** — MCAP이 메시지 정의를 파일 안에 넣고 다니기 때문이다.
`pip install mcap mcap-ros2-support`면 끝이다(Ubuntu 24.04는 `--break-system-packages`).

## 구조 — 공용과 프로젝트를 가른다

```
core/                로봇도 과제도 모른다
  rosmsg.py            ROS 2 .msg → MCAP 스키마 본문
  mcap_read.py         MCAP → 시행 기록 (판정 규칙은 project.py에서 받아 쓴다)
  regress.py           세대별 누적 · 순열 검정 회귀 탐지
  lerobot_out.py       LeRobotDataset v3.0 내보내기
  build.py             템플릿 + JSON → 한 장짜리 HTML
projects/
  capstone-pick/       SO-101 pick-and-place
    project.py           **이 프로젝트의 전부** — 관절·프레임·판정 규칙·화면 구성
    dashboard.tpl.html   화면 (프로젝트마다 아예 다르게 짜도 된다)
    to_mcap.py           옛 JSONL → MCAP (이 프로젝트 전용 변환)
    record_mcap.sh       ros2 bag record로 직접 MCAP 받기
  slam/                다음 프로젝트 자리 — projects/slam/README.md 참고
docs/                  인수인계·조사 원자료·다른 프로젝트를 태울 때 채우는 양식
legacy/                자체 JSONL을 직접 읽던 옛 경로. 참고용
```

`core/`에는 로봇 이름도 판정 규칙도 없다. 다른 로봇으로 옮길 때 **고치는 곳은
`projects/<이름>/project.py` 하나**이고, 그게 사실인지는 `check`가 확인한다.

## 데이터가 들어오는 길

```
추적 JSONL ──to_mcap.py──┐                        ┌─read──▶ runs.json · run_frames.json
                         ├─▶ 감시 디렉터리/*.mcap ─┤
ros2 bag record ─────────┘                        └─trend─▶ trend.json
                                                              │ build
                                                              ▼  dashboard.html
```

입구가 MCAP 하나다. 변환본이든 `ros2 bag record` 산출물이든 리더는 구별하지 않고,
`ros2 bag info`·Foxglove로도 열린다. 받는 방식은 Foxglove Agent와 같은 계약이다 —
**완성된 파일이 디렉터리에 떨어져 있으면 집는다.** 반쯤 쓰인 파일을 집지 않도록
`.mcap`으로 이름이 바뀐 것만 본다.

## 자세한 것

- `docs/HANDOFF.md` — 지금 상태, 검증한 숫자, 이미 당한 함정, 남은 일
- `docs/RESEARCH_CLAIMS.md` — 시장·기술 조사 원자료 (출처 23건 · 주장 115개)
- `projects/slam/README.md` — 새 프로젝트를 태우는 순서
