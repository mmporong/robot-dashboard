# SO-101 Bench Telemetry — 인계 (2026-08-21)

실기 기록 대시보드를 **새로 만들어 붙인 상태**다. 돌아가고 검증도 끝났다.
이 문서는 화면을 더 낫게 만드는 다음 사람을 위한 것이다.

같은 디렉터리에 **화면이 둘** 있다. 헷갈리면 안 된다:

| 파일 | 무엇 | 데이터 |
| --- | --- | --- |
| `panel.html` + `panel_server.py` | **라이브** — 지금 팔을 움직인다 | 서보에서 직접 (HTTP 8765) |
| `dashboard.tpl.html` + `read_runs.py` | **기록** — 끝난 시행을 되돌려 본다 | `~/so101_tools/media/` · `~/so101_datasets/` |

이 인계는 **기록 쪽만**이다. 라이브 패널은 실물 팔을 움직이는 코드라 이 작업 범위 밖이다.

---

## 돌리는 법

```bash
~/miniforge3/envs/lerobot/bin/python ~/robot-dashboard/dash.py so101-arm all
# read(기록 수집 + 영상 재인코딩) → build(템플릿+JSON → dashboard.html)
```

`python3` 를 그냥 쓰면 안 된다 — 영상 재인코딩에 PyAV 가 필요한데 시스템 python3 에는 없다.

빌드에 **1분 45초**쯤 걸린다(대부분 영상 인코딩). 화면만 고칠 때는 `build` 만 돌리면 즉시 끝난다:

```bash
~/miniforge3/envs/lerobot/bin/python ~/robot-dashboard/dash.py so101-arm build
```

브라우저 확인은 `file://` 이 막혀 있으니 정적 서버로:

```bash
cd ~/robot-dashboard/projects/so101-arm && python3 -m http.server 8790 --bind 127.0.0.1
# → http://127.0.0.1:8790/dashboard.html   (확인 끝나면 PID 로 종료, pkill -f 금지)
```

## 파일

| 파일 | 역할 |
| --- | --- |
| `project.py` | 이 프로젝트의 설정 전부 — 채널·예산·SHOW·판정 상수 |
| `read_runs.py` | 미디어 디렉터리 → `runs.json` · `datasets.json` (영상 재인코딩 포함) |
| `dashboard.tpl.html` | 화면. `/*__RUNS__*/` · `/*__DATASETS__*/` 자리에 JSON 이 치환된다 |
| `dashboard.html` | 생성물 (13MB) — **직접 고치지 말 것**, 다음 빌드에 덮어써진다 |

`core/` 는 손대지 않았다(공용). `dash.py` 만 세 군데 고쳤다:
- `check` 의 필수 키를 프로젝트가 `REQUIRED` 로 선언 가능하게 (기존 목록은 ROS/MCAP 전제라 TF 프레임을 요구했다)
- `read` 단계에서 프로젝트에 `read_runs.py` 가 있으면 그쪽을 쓴다 (`mcap_read` 라는 이름에 mp4 리더를 넣지 않으려고)
- `TREND = False` 인 프로젝트는 `all` 에서 trend 를 건너뛴다

## 데이터가 어디서 오나

`run_demo.sh` 한 번 = 시행 하나 = **파일 묶음**이다. MCAP 한 덩어리가 아니다.

```
~/so101_tools/media/<날짜>/demo_<시각>_rgb.mp4       정면 (뎁스캠 컬러)
                            _wrist.mp4     손목캠
                            _depth.mp4     깊이
                            _sim.mp4       MuJoCo 미러
                            _screen.mp4    패널 화면
                            _state.csv     t_s·torque·관절각6·온도6 (전압은 최근 판만)
```

`_realtime` 접미사가 붙은 판이 있으면 **그쪽을 쓴다**. 무접미 판은 mpjpeg 를 벽시계
타임스탬프 없이 받아 2.5배속으로 굳은 기록이다(2026-08-20 실측). 이 경고는
`rgb`·`wrist`·`depth` 에만 해당한다 — `sim`·`screen` 은 처음부터 실시간이라 대상이 아니다.

`~/so101_datasets/<이름>/` 는 라이브 패널의 레코더가 만드는 LeRobot 표준 데이터셋이다.
현재 0개(팔이 꺼져 있어 아직 못 찍었다) — 목록이 비어 있어도 정상이다.

## 지금 되는 것 (브라우저로 확인 완료)

- 5채널이 **하나의 시간축**으로 동시 재생 · 시크 · 일시정지
- 관절각·온도 차트, 재생 위치 세로 커서, 범례에 그 시점 값
- 파지 시도(그리퍼 열림→닫힘 전이)를 시간축에 빨간 세로선으로
- 계측 카드: 길이·채널·최고온도와 시점·파지 시도·그리퍼 최소각·전압 최저·토크 꺼진 시각
- 시행 칩 9개 전환 (영상 없는 시행은 계측만)
- 콘솔 에러 0

## 개선 후보 (우선순위 순, 판단은 넘겨받는 쪽에서)

1. **용량**. 지금 13MB(영상 9.9MB)에 예산 상한 11MB 로 눌려 crf 34·해상도 75% 까지
   내려가 있다. 채널을 켜고 끄는 UI 를 두고 기본은 2채널만 싣거나, 시행별로 별도
   파일에 담아 지연 로딩하면 화질을 되찾을 수 있다. `project.py` 의
   `VIDEO_BUDGET_MB` · `VIDEO_MAX_W` · `VIDEO_FPS` 가 레버다.
2. **단계 타임라인**. 지금 시간축에 찍히는 것은 파지 시도뿐이다. 실제 데모는
   `unfold → pick → drop → park` 네 단계인데 그 경계가 화면에 없다. `run_demo.sh` 가
   단계 전환 시각을 파일로 남기게 하고(지금은 stdout 뿐) 타임라인에 띠로 그리면
   "어느 단계에서 길어졌나"가 한눈에 보인다.
3. **시행 비교**. 두 시행을 나란히 놓고 같은 시각의 관절각을 겹쳐 그리는 화면.
   "어제는 됐는데 오늘은 왜"를 보는 데 이게 제일 빠르다.
4. **실패 사유**. `run_demo.sh` 는 실패 단계 이름을 알고 있는데(`fail` 변수) 기록에
   안 남긴다. 남기면 시행 칩에 실패 배지를 달 수 있다.
5. **데이터셋 상세**. 지금은 목록(회차·프레임·fps)뿐. 에피소드별 길이·태스크 문구,
   레코더가 계측하는 `dup_pct`(상태 갱신이 기록 주기보다 느려 같은 값이 쌓인 비율)를
   실으면 학습에 쓰기 전 품질 판단이 화면에서 된다.
6. **모바일 폭**. 차트는 `@media (max-width:900px)` 로 1열이 되지만 영상 그리드와
   상단 칩 줄은 좁은 폭에서 검증하지 않았다.

## 건드릴 때 지켜야 할 것

- **판정 라벨을 자동으로 붙이지 말 것.** 실기에는 Gazebo 실좌표 같은 심판이 없고,
  로그의 성공 문구가 실물과 어긋난 전례가 있다(2026-08-19: 로그는 PICK_SUCCESS,
  실물은 헛집음). 계측된 사실만 싣고 성공/실패는 사람이 영상을 보고 붙인다.
  이 원칙이 이 대시보드의 정체성이다.
- **`<meta charset="utf-8">` 를 지울 것 없다.** 없으면 한글이 전부 깨진다.
  같은 결함이 `capstone-pick/dashboard.tpl.html` 에도 있어 거기도 넣어 뒀는데,
  그쪽 `dashboard.html` 은 **아직 재빌드하지 않았다** — 필요하면
  `dash.py capstone-pick build` 로 반영하면 된다(기존 산출물을 임의로 덮지 않으려고 남겨 뒀다).
- **실물 팔을 움직이지 말 것.** 이 작업은 기록 화면이라 팔이 필요 없다. 팔 이동은
  건별로 사용자 승인이 필요하다.
- **`SHOW` 에 없어도 가장 최근 시행은 항상 영상까지 실린다**(`SHOW_LATEST`). 데모를
  새로 찍으면 그 자리에서 화면에 뜨게 하려는 의도다. 이걸 끄면 매번 손으로 고쳐야 한다.
- `run_demo.sh` 의 `finalize` 가 끝나면 백그라운드로 이 대시보드를 다시 빌드한다
  (`DEMO_NO_DASH=1` 로 생략). 훅을 고치면 그쪽도 같이 봐야 한다.

## 커밋 상태

`feat/so101-arm` 브랜치에 **미커밋**이다. 오늘 이 세션에서 만진 것:

```
 M dash.py
 M projects/capstone-pick/dashboard.tpl.html      (charset 한 줄)
 M projects/so101-arm/panel.html                  ← 라이브 패널(미러·데이터셋 UI). 기록 쪽과 무관
 M projects/so101-arm/panel_server.py             ← 같음
 M projects/so101-arm/tools/*                     ← 같음 (실물 제어 코드)
?? projects/so101-arm/project.py                  ← 기록 대시보드
?? projects/so101-arm/read_runs.py                ← 기록 대시보드
?? projects/so101-arm/dashboard.tpl.html          ← 기록 대시보드
?? projects/so101-arm/dashboard.html              ← 생성물 13MB (커밋 여부는 사용자 판단)
```

스테이징은 **파일 단위로**. `git add .` 는 다른 세션의 미커밋 변경을 빨아들인다.
