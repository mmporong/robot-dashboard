# 관제 대시보드 — 인수인계

**한 줄**: SO-101 pick-and-place 실행 기록을 되감아 "물체가 언제 물리고 들리고 빠졌는지"를
실좌표로 다시 판정하는 단일 페이지 관제 화면.

**발행본**: https://claude.ai/code/artifact/7e10dd4c-b1ab-41f1-b0bd-6ce55f8c1067
(같은 URL 유지하려면 `Artifact` 호출 때 `url`로 이 주소를 넘길 것. 안 넘기면 새 아티팩트가 생긴다)

**작업 위치**: `~/robot-dashboard`
**데이터 인박스**: `~/capstone_tools/mcap/` (MCAP) · **옛 원본**: `~/capstone_tools/logs/`
(`mcap_backup_20260813/`에 이관 전 상태 백업) · **시뮬 코드**: `~/jdamr_cube_ws/src/jdamr_cube_ros/capstone_pick`

---

## 1. 파일 구성

리포 구조는 최상위 `README.md`에 있다. 여기는 **캡스톤 pick 프로젝트에서 무엇이
어디 있는지**만 적는다. 경로는 `~/robot-dashboard/` 기준이다.

| 파일 | 역할 |
|---|---|
| `core/mcap_io.py` | **중립 층.** 파일 열기·시각 조회·TF 합성·프레임 굽기·용량 예산. 로봇도 과제도 모른다 |
| `core/regress.py` | 세대별 누적 · 순열 검정 회귀 탐지. 프로젝트 리더의 `run_of()` 하나만 부른다 |
| `core/lerobot_out.py` | MCAP → **LeRobotDataset v3.0**. lerobot 쓰기 API를 그대로 쓴다 (lerobot 환경 필요) |
| `core/rosmsg.py` | ROS 2 `.msg` → MCAP 스키마 본문. 쓰는 쪽에서만 쓴다 |
| `core/build.py` | 템플릿 + JSON 4종 → `dashboard.html` |
| `projects/capstone-pick/mcap_read.py` | **이 과제의 읽기.** 무엇을 단계로 보고 무엇을 성공으로 볼지 |
| `projects/capstone-pick/project.py` | 설정 — 관절 이름·프레임·판정 규칙·화면에 세울 시행 |
| `projects/capstone-pick/dashboard.tpl.html` | 화면. 데이터 자리는 `/*__RUNS__*/` `/*__CHAIN__*/` `/*__FRAMES__*/` `/*__TREND__*/` |
| `projects/capstone-pick/to_mcap.py` | **레거시 변환기.** 추적 JSONL·프레임 JSON → 시행당 MCAP. 세대(epoch)를 파일에 적는다 |
| `projects/capstone-pick/record_mcap.sh` | 시뮬 1회를 `ros2 bag record`로 직접 MCAP에 담아 인박스에 놓는다 |
| `projects/capstone-pick/extract_chain.py` | URDF → `urdf_chain.json` (관절 18 + 손가락 패드 6) |
| `legacy/` | 자체 JSONL을 직접 읽던 옛 경로. 참고용 |
| `docs/PROJECT_PROMPT_TEMPLATE.md` | 다른 프로젝트를 태울 때 채우는 9절 양식 |
| `docs/RESEARCH_CLAIMS.md` | 시장·기술 조사 원자료 (출처 23건 · 주장 115개) |

**2026-08-14에 구조가 갈렸다.** 처음엔 `core/mcap_read.py` 하나가 읽기를 다 했는데,
SLAM을 태우려 하자 그것이 `/joint_states`와 팔 관절 이름을, 판정이 `/gt/trash`를
전제해서 **import조차 되지 않았다.** "core는 로봇도 과제도 모른다"고 적어 뒀던 것이
사실이 아니었다. 중립인 부분(`mcap_io`)과 아닌 부분(프로젝트 `mcap_read`)을 파일로
갈랐고, 갈라도 결과가 같음을 `runs.json`·`trend.json` 완전 일치로 확인했다.

**빌드**: `cd ~/robot-dashboard && python3 dash.py capstone-pick all`
(구조가 `core/` + `projects/<이름>/`으로 갈렸다 — 리포 README 참고)

**LeRobot 내보내기**(선택): `projects/capstone-pick/`에서
`PYTHONPATH="$PWD:../../core" ~/miniforge3/envs/lerobot/bin/python ../../core/lerobot_out.py`
→ `~/capstone_tools/lerobot/<세대>/`. 대시보드 빌드와는 무관하다.

**데이터 흐름** — 입구가 MCAP 하나다. 어느 경로로 들어왔든 리더는 구별하지 않는다.

```
추적 JSONL ──to_mcap.py──┐                        ┌─read──▶ runs.json · run_frames.json
                         ├─▶ ~/capstone_tools/mcap/*.mcap ─┤            (화면에 세운 7시행)
ros2 bag record ─────────┘      (감시 디렉터리)   └─trend─▶ trend.json
                                                                   (전량 누적·회귀)
                                        runs.json + run_frames.json + trend.json + urdf_chain.json
                                                          │  build.py
                                                          ▼
                                                    dashboard.html
```

읽는 쪽은 **ROS가 없어도 돈다** — MCAP이 메시지 정의를 파일 안에 넣고 다닌다.
필요한 것은 `pip install mcap mcap-ros2-support`뿐이다(Ubuntu 24.04는 `--break-system-packages`).
쓰는 쪽(`to_mcap.py`)만 `.msg` 원문을 읽느라 ROS 설치가 필요하다.

---

## 2. 지금 상태 — 무엇이 되어 있나

**화면** (한 화면 안에 전부, 페이지 스크롤 없음)
- 판정 배너 → 2×2 타일(위: 로봇 자세·전방캠 / 아래: 평면도·손목캠) → 상태 칸 → 타임라인
  (2026-08-13 사용자 지시로 손목캠과 평면도를 맞바꿨다. `.tiles` CSS는 순서에 의존하지
  않으니 마크업 순서만 바꾸면 된다)
- **분석은 오른쪽 서랍**으로 열린다. 열려도 재생이 계속 돌고, 상태 전이표가 재생 헤드를 따라간다
- 연속 시간 재생(rAF + smoothstep 보간). 속도는 "전체를 몇 초에" 기준 — 느리게 45초 / 보통 22초 / 빠르게 10초

**데이터**: 시행 7개 — 팔 단독 성공 3(A1·A2·A3) + 주행 실패 4(#14·#10·#17·#13).
인박스(`~/capstone_tools/mcap/`)에는 MCAP 78개가 들어 있다 — 주행 72 + 팔 단독 1세대 3
+ 팔 단독 2세대 2 + `ros2 bag record` 실기록 1. 화면에 세우는 7개는 `project.py`의
`SHOW`가 고르고, **나머지는 `core/regress.py`가 누적·회귀로 쓴다**(8절).

**MCAP 전환에서 드러난 것**: 옛 파이프라인은 단계와 카메라 프레임의 0초를 **따로**
잡았다. 녹화는 무대를 세우는 동안 이미 돌고 있어서 첫 단계보다 4.5초 앞서 시작하는데
(실측: 시행1 +4.53s · 시행2 +4.48s · 시행3 +4.49s), 양쪽을 각자 0으로 맞추자 팔 단독
전 시행에서 **영상이 관절보다 4.5초 앞선 채** 겹쳐 재생됐다. bag은 시계가 하나라
옮기는 순간 드러났고, 리더가 두 계열을 같은 값만큼 밀어 고쳤다.

**검증한 것** (숫자는 실측)
- MCAP 경로가 옛 경로와 **동일한 결과**를 낸다 — 시행 7개 × 전 단계 × 전 필드에서
  차이 0건(판정·거리·주장·관절·죠·실좌표 전부). 위 4.5초 정렬만이 의도한 차이다
- 생성한 MCAP을 `ros2 bag info`가 읽는다 — 토픽 10개·메시지 613개·시간 140.23s가
  제대로 잡힌다. 우리 리더만 읽을 수 있는 파일이 아니라는 뜻이다
- 정기구학이 ROS `tf2`와 **0.5mm 이내** 일치 (TF 출력이 소수 셋째 자리까지라 그 이하는 확인 불가)
- FK 대 기록된 죠 좌표: 중앙값 0.135mm · 84%가 1mm 미만. 67.7mm짜리 예외 1건은 팔이
  움직이는 중 관절값과 TF 조회 시각이 어긋난 자리(#13 t=168.4)
- 렌더 1048회(계단 사이 보간 지점 포함) × 서랍 닫힘/열림 무사통과
- 재생 826프레임/5초 · 중앙 6ms · 최악 8.6ms · 33ms 초과 0회
- 1366×768과 1920×1080에서 전 시행·전 시점 페이지 스크롤 0, 상태 칸 내부 스크롤 0
  (타일 교체·서랍 「데이터 출처」 추가 뒤 **실제 브라우저로 다시 확인**했다 — 두 해상도에서
  시행 7개를 전부 눌러 가로·세로 스크롤 0, 콘솔 오류는 favicon 404 하나뿐. 서랍은 자체
  스크롤 2658px이라 늘어나도 바깥 레이아웃을 밀지 않는다)

**검증 방법**: 브라우저 확인은 **charset을 명시한 서버**로 띄워야 한다 — 기본
`http.server`는 charset을 안 붙여 한글이 windows-1252로 깨진다. `guess_type`을 덮어
`; charset=utf-8`을 붙이는 20줄짜리면 된다.

---

## 3. 조사 결론 — 방향이 바뀌었다

원자료는 `RESEARCH_CLAIMS.md`. 반박 검증 단계가 워크플로 중단으로 못 돌아 한동안
"출처에서 뽑기만 한" 상태였다. **결론이 걸린 두 개는 2026-08-13에 소스로 직접 확인했다.**

- **LeRobot v0.6.0 `lerobot.rewards` — 사실.** `huggingface/lerobot`의 `src/lerobot/rewards/`에
  `classifier`(HIL-SERL) · `sarm` · `robometer` · `topreward` + `factory.py` · `pretrained.py`가
  실재한다. v0.6.0은 2026-07-06, 최신은 v0.6.1(2026-08-03)이고 그 뒤로도 리워드 관련 커밋이
  계속 들어온다 — 식은 API가 아니다
- **AllenAI 하네스 회귀 테스트 Planned — 사실.** `docs/architecture.md`의 Planned Features에
  `Reference score comparison: Regression testing against known model+benchmark scores.`가
  지금도 남아 있다. 코드 검색으로 교차확인: `reference_score` 0건. 리포는 살아 있다
  (마지막 push 2026-08-12 · 별 527). **살아 있는 리포에서 아직 미구현**이라 근거로 더 세다
- 다만 같은 Planned 목록의 "Video / trajectory recording"은 이미 낡았다
  (`tests/test_recording_sqlite.py`가 있다). **저 문서는 뒤처진다** — 근거로 인용할 땐
  문서보다 코드 검색 쪽을 앞세울 것

**틀린 가설**: "시각화는 피하고 판정 레이어로 가라"
판정 레이어는 빈자리가 아니다. LeRobot v0.6.0이 `lerobot.rewards` 통합 API를 신설하며
"성공 판정은 로봇 학습 루프의 빠진 조각"이라 명시했고, Robometer(100만 궤적 사전학습)와
TOPReward(완전 제로샷 VLM)가 무료로 그 일을 한다. NVIDIA Isaac Lab-Arena, Foxglove BYOS의
`evaluation`, Roboto Agents, Antioch($8.5M 시드), Lightwheel까지 동시에 들어와 있다.
**판정 알고리즘 자체가 커모디티화됐다.**

**시뮬 판정의 한계가 수치로 있다**: 시뮬-실기 상관 ρ≈0.59 (sim2sim은 0.97).
Hugging Face도 "시뮬은 contact-rich 조작엔 부적합, 알고리즘 평가용으로만"이라고 못 박았다.

**그래도 진짜 비어 있는 좁은 틈**
- AllenAI 평가 하네스(18 벤치마크 × 40+ VLA)조차 **회귀 테스트가 Planned Features**다.
  평가 *실행*은 해결됐는데 **실행 간 비교·회귀 탐지는 아직**이다
- 그 하네스의 기록은 SQLite + 선택적 비디오 수준이고 ROS/rosbag/MCAP 언급이 없다 —
  ROS 2 실기·Gazebo 실행 기록 쪽은 그 생태계 밖이다
- 리더보드 점수는 대부분 저자 자가보고라 **독립 재판정은 미해결**이라고 스스로 경고한다

**상품성 판단**: 지금 그대로는 상품이 아니다.
시각화·리플레이는 무료 커모디티(Foxglove가 Basic Seat 무료화 + 진입 티어 $126→$20, 84% 인하,
누적 조달 $58M+). "관절+카메라 동기 프레임 리플레이"는 이미 상용 기능 목록에 있다.
유료 전환은 RBAC가 아니라 SAML SSO·감사로그·자체 배포에서 일어나는데 전부 없다.
학술 계정은 Foxglove가 $0에 1TB·10 devices·무제한 사용자를 준다.
그리고 **READY Robotics 패턴**($41.5M 조달·Rockwell 투자에도 2024-08 폐업)에 정확히 걸린다 —
실패 메커니즘이 "기존 것 위에 얹는 추가 구매라 증분 이익 기준을 못 넘김"이었다.

**돈을 내게 만드는 서사**는 판정 정확도가 아니라 **디버깅 시간 단축**이다
(Dexterity: Foxglove 도입으로 개발시간 20%↓, 연 $150k 절감).

---

## 4. 다음에 할 일 — 순서대로

기준은 **"이 분야 엔지니어가 보고 실력을 인정할까"**다. 기존 도구와 겹쳐도 상관없다
(사용자 방침: 포트폴리오가 1순위, 빈자리 찾기로 방향 틀지 말 것).

**~~1. MCAP 읽기~~ — 2026-08-13 완료.** 입구가 MCAP 하나가 됐다(위 데이터 흐름 참고).
옛 JSONL은 `to_mcap.py`로 옮겼고, 리더가 변환본과 `ros2 bag record` 산출물을
구별 없이 읽는다. 결과는 옛 경로와 차이 0건. 압축은 MCAP 자체의 **청크 레벨 zstd**를 쓴다 —
전체 gzip은 인덱스를 파괴해 부분 추출이 불가능해진다.

**~~2. N회 누적 + 회귀 탐지~~ — 2026-08-13 완료.** `core/regress.py` · 서랍의 「누적 · 회귀」.
세대(epoch) 안에서만 견주고, 판정에는 순열 검정으로 유의확률을 붙인다. 8절 참고.

**~~3. LeRobotDataset 스키마로 내보내기~~ — 2026-08-13 완료.** `core/lerobot_out.py`. 9절 참고.

1. **규모 이야기** — 로봇은 초당 1GB를 기록하는데 현장 업로드는 10~100Mbps다. 2~3 자릿수
   격차라 선택적·부분 업로드가 최적화가 아니라 구조 요건이다. 지금 화면엔 이 얘기가 없다.
   **실측 재료가 생겼다**: 팔 단독 1회를 6토픽으로 받아 적으니 **252초에 163MB**다
   (0.65MB/s). 같은 시행을 변환본으로 담으면 0.6MB — **270배** 차이다. 무엇을 버렸길래
   그런지가 곧 "부분 업로드"의 내용이다(관절 227Hz → 단계 18개, 카메라 4,412장 → 358장).
2. **실좌표를 ROS 토픽으로 발행하기** — 지금 실좌표는 `gz model -p` CLI로 긁어 추적
   스크립트가 받아 적는다. 그래서 `ros2 bag record`만으로 만든 bag에는 **판정 근거가 없다**
   (아래 검증에서 실제로 "판정 불가"가 나왔다). 작은 노드 하나가 `/gt/cube`·`/gt/robot`·
   `/gt/trash`를 `geometry_msgs/PoseStamped`로 내보내면 기록 한 번으로 판정까지 닫힌다.

**만들지 말 것**: 자체 데이터 포맷(LeRobot 논문이 ROS bag을 "파편화 원인"으로 분류),
3D 렌더러 고도화(Rerun은 wgpu/WebGPU에 Arrow 컬럼 청크로 100x 개선을 이미 했다),
실시간 스트리밍 인프라, 가격·결제.

---

## 5. 함정 — 이미 당한 것들

- **`gz model --list`의 첫 `pick*` 모델을 집으면 안 된다.** 월드에 큐브가 셋이면 비전이 찾는
  색과 다른 걸 잡는다. `armonly_stage.py`의 `cube_name(color, models)`가 색으로 고른다
- **방해 큐브를 안 치우면 차체가 그 위에 올라탄다.** 실측: 3cm 큐브 하나에 3.7° 기울고
  0.36m 앞의 죠가 23mm 어긋나 5회 연속 빈손 파지. 로그엔 "물지 못함"만 남아 원인이 안 보였다.
  `armonly_stage.py`가 이제 치우고 **차체 수평까지 검사**한다(임계 2°; 이 차체는 평소에도 1.1° 기운다)
- **`_trace`의 대상 모델 이름**이 `pick_object_{색}`으로 만들어진다. 무대 실제 이름과 다르면
  큐브 좌표가 전부 null로 기록되고 "판정 불가"로 읽힌다. `armonly_pick.py`가 `n._trace_target`을 덮어쓴다
- **카메라 콜백을 나중에 갈아 끼울 수 없다** — 구독 객체가 원래 바운드 메서드를 쥐고 있다.
  녹화는 **구독을 하나 더** 거는 방식이다(`Recorder` 클래스)
- **`pkill -f`·인라인 `pgrep -f` 금지** — 패턴이 자기 셸 argv에 걸려 스스로 죽는다(실제로 겪음).
  패턴은 반드시 스크립트 파일 안에 둘 것
- **RTF < 0.9면 측정 무효.** 코드가 실시간 대기에 의존한다
- **작업 후 `~/capstone_tools/sim_down.sh`** (팬 안전). 이 스크립트의 gz 패턴이 `/gz sim`이라
  16시간 동안 한 번도 안 잡히던 버그를 고쳤다(지금은 `gz sim -r`/`gz sim -g` + TERM 후 KILL 확인)
- **아티팩트는 밝은 색 고정.** 다크 대응하지 말 것

**MCAP 쪽에서 새로 당한 것**

- **`dict`로 메시지를 넘기면 `values` 필드가 깨진다.** `mcap_ros2` 인코더는 필드를
  `hasattr`로 먼저 찾는데 `dict`엔 `values` 메서드가 이미 있다 —
  `DiagnosticStatus.values`가 그 메서드로 잡혀 "not an array"로 죽는다.
  `to_mcap.py`의 `msg()`가 트리 전체를 `SimpleNamespace`로 세워 피한다
- **리더를 직접 만들 땐 해독기를 물려야 한다.** `make_reader(f)`만으로 만든 리더를
  `read_ros2_messages`에 넘기면 `DecoderNotFoundError`가 난다.
  `make_reader(f, decoder_factories=[DecoderFactory()])`로 세울 것
- **스키마 본문을 미리 다 등록하지 말 것.** 정의 하나가 2~4KB라 짧은 주행 시행에서는
  본문보다 스키마가 커진다(30KB → 19KB). 실제로 쓴 토픽만 등록한다
- **`stamp`의 `nanosec`은 `uint32`다.** 시각이 음수면 `struct.error`로 죽는다 —
  기준 시각을 잘못 잡으면 여기서 터진다
- **`set -u`를 쉘 스크립트에 쓰지 말 것.** ROS의 `setup.bash`가 미설정 변수를
  참조해 그 자리에서 죽는다(`AMENT_TRACE_SETUP_FILES: unbound variable`)
- **bag은 SIGINT로 끝내야 한다.** TERM으로 끊으면 요약·인덱스가 없는 파일이 남는다.
  `systemctl --user kill --signal=SIGINT capstone-bag`
- **반쯤 쓰인 파일을 인박스에 두지 말 것.** staging에 기록하고 끝난 뒤 `mv`로 옮긴다
- **`armonly_pick.py`는 `armonly_trace.jsonl`에 append한다.** 그냥 돌리면 보존하기로 한
  옛 기록에 새 시행이 섞인다. `record_mcap.sh`가 원본을 옆으로 치웠다 되돌린다

---

## 6. 데이터 재생성

```bash
# 시뮬 (headless — 화면 켜면 RTF 떨어져 시행이 망가진다)
source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=lim-capstone
ros2 launch jdamr_cube_gazebo gui:=false \
  world:=$(ros2 pkg prefix jdamr_cube_gazebo)/share/jdamr_cube_gazebo/worlds/room.world

# 팔 단독 3회 + 녹화 (rec 인자)
python3 ~/jdamr_cube_ws/src/jdamr_cube_ros/capstone_pick/tools/armonly_pick.py 3 0 rec
#   → ~/capstone_tools/logs/armonly_trace.jsonl · armonly_frames.json

# 기록을 MCAP으로 바로 받기 (시뮬이 떠 있는 상태에서)
~/robot-dashboard/record_mcap.sh live2      # → ~/capstone_tools/mcap/live2.mcap

cd ~/robot-dashboard
python3 dash.py capstone-pick all
python3 dash.py capstone-pick probe ~/capstone_tools/mcap/live2.mcap   # 이 파일에서 뭐가 읽히나
```

## 7. 진짜 bag으로 한 검증 — 그리고 거기서 나온 시뮬 문제

2026-08-13에 `ros2 bag record`로 팔 단독 1회를 담아(`live1.mcap` · 163MB · 252초)
리더에 물렸다. **리더 쪽은 통과**했다 — 관절 57,418 · TF 59,704 · odom 2,823 ·
카메라 4,412장을 읽고, 단계 표시가 없는 bag이라 1Hz 재표집으로 252단계를 세웠고,
그리퍼를 이름으로 찾아 252/252를 채웠다. TF 체인 합성은 `tf2`와 **0.00000mm** 일치
(3칸짜리 체인 포함 9개 대조, tf2가 실패하는 자리에서 똑같이 실패).

**그런데 그 시행 자체는 무효다.** 그리고 원인 세 개가 시뮬 쪽에 있다 — 캡스톤 세션 몫이다.

- **`/clock`이 ROS로 안 넘어온다.** gz는 `/clock`을 정상 발행하는데(`gz topic -e -t /clock`으로
  확인) ROS 쪽 `/clock`은 발행자 1·구독자 5인 채 **메시지가 0**이다. 다른 토픽(관절·이미지·tf)은
  같은 브리지로 잘 넘어온다 — `/clock`만 막혔다
- **그래서 `/tf`에 팔이 없다.** `robot_state_publisher`는 `use_sim_time=True`인데 시각을 못 받아
  **동적 TF를 하나도 발행하지 않는다.** `/tf`에 있는 자식은 `base_footprint`·좌우 바퀴 셋뿐이고,
  움직이는 팔 링크 6개(shoulder_pan·shoulder_lift·elbow_flex·wrist_flex·wrist_roll·gripper)는
  `/tf`에도 `/tf_static`에도 없다. **옛 기록의 `jaw`가 tf2에서 나왔다는 점을 생각하면,
  예전엔 되던 것이 지금 깨져 있다는 뜻이다**
- **RTF가 0.13이다.** 16코어·부하 3.3·59°C·4GHz로 경합도 스로틀링도 아니고 `gz sim`이 한 코어
  (108%)에 묶여 있다. 0.9 게이트를 한참 밑돌아 실시간 대기에 의존하는 코드가 무너진다 —
  이 시행이 실패한(0/1 · 큐브가 통 중심에서 +197,+277mm · 높이 269mm) 직접 원인으로 보인다.
  **녹화 탓이 아니다**: 압축 카메라 구독 유무로 A/B를 뜬 결과 0.135 → 0.134 → 0.130으로 차이가 없다

판정은 로그가 아니라 **실좌표**로 한다 — `gz model -m <name> -p`.
개구부는 136mm각(반폭 68mm), 벽 상단 0.090m. 물체 중심이 개구부 안이고 높이가 벽 상단보다
낮아야 통 안이다.

---

## 8. 누적 · 회귀 — 무엇을 어떻게 판정하나

한 시행을 되감아 보는 것과 **시행들을 견주는 것**은 다른 일이다. 평가를 *돌리는* 쪽은
이미 무료 오픈소스가 채웠지만 실행 간 비교는 선도 하네스에서도 Planned다(3절). 여기가 그 자리다.

### 설계 원칙 세 가지

1. **세대(epoch)를 넘어 견주지 않는다.** 무대 배치가 바뀌면 같은 잣대가 아니다.
   세대는 **파일 메타데이터**에서 읽는다 — id로 짐작하면 `ros2 bag record` 파일에는 쓸 수 없다.
2. **성공률만 보지 않는다.** 이 기록에서 실제로 일어난 사고는 성공률 하락이 아니라
   **판정 근거의 소실**이었다. 성공률만 보면 "시행이 줄었다"로 읽히고 조용히 지나간다.
3. **탐지기를 먼저 의심한다.** `python3 dash.py capstone-pick selftest`로 귀무모형을 돌린다.

### 세대

| 세대 | 무엇 | 기록 |
|---|---|---|
| `arm-v1` | 월드에 큐브가 **하나뿐이던** 무대 — 방해 큐브가 없어 차체가 올라탈 일이 없었다 | 3시행 · 통 안 3 |
| `arm-v2` | 2026-08-13 무대 개편(캡스톤 세션) — 방해 큐브를 벽 속 (3.0, 2.4)에 버리던 것을 고치고 세 큐브를 팔이 닿는 자리에 모았다 | 2시행 / **3회 시도** · 통 안 2 |
| `drive-v1` | 이동·접근까지 포함 | 72시행 · 판정 가능 40 · 통 안 **0** |

`arm-v2`의 원본은 `~/capstone_tools/logs/armonly_trace.jsonl`, `arm-v1`은
`logs/mcap_backup_20260813/armonly_trace.jsonl`이다 — **파일명이 같고 내용이 다르므로**
메타데이터의 `source`에 경로째 적는다.

### 탐지 방법과 그 한계

후보 경계마다 앞뒤 평균 차를 재고 가장 큰 것을 고른 뒤, **순서만 무작위로 섞어**
같은 통계를 4,000번 다시 잰다. 관측값 이상이 나온 비율이 p다. 최대값 통계라 후보를
여럿 본 것에 대한 보정이 이미 들어 있다. p는 `(초과+1)/(시행+1)` — 0회를 p=0으로 적으면
"절대 확실"이 되는데, 4,000번 못 봤다는 것과 불가능하다는 것은 다르다.

자체 시험 결과(`--selftest`, 각 300회):

| 계열 | p<0.05 비율 |
|---|---|
| 변화 없음 (동전 던지기 40회) | **6.0%** ← 유의수준 5%와 맞는다 |
| 변화 없음 (전부 0) | 0.0% |
| 진짜 계단 (1 → 0) | 100.0% |
| 약한 계단 (0.7 → 0.3) | **34.0%** |

마지막 줄이 중요하다. **"변화 없음"은 변화가 없다는 증명이 아니다** — 40시행으로는
0.7에서 0.3으로 떨어지는 변화조차 셋에 하나만 잡힌다. 화면에도 그렇게 적어 두었다.

### 지금 나오는 결론

- **`drive-v1` 판정 가능: `drive_43`부터 100% → 0% · p=0.0002 — 회귀.**
  5절의 대상 모델 이름(`pick_object_{색}`) 불일치가 그 시점부터 계속 걸려 큐브 실좌표가
  통째로 안 남았다. 로그에는 아무 표시가 없다. **이 도구가 아니면 안 보이는 종류의 고장이다.**
- `drive-v1` 거짓 성공: 가장 큰 격차 27%p · **p=0.2342 — 유의하지 않다.** 10시행 단위로
  2→2→0→0이라 줄어드는 것처럼 보이지만 순서를 섞어도 이만한 격차가 흔히 나온다.
- `drive-v1` 통까지 거리: 격차 426mm · **p=0.0747 — 유의하지 않다.** 아슬아슬하다.
- `drive-v1` 실좌표 성공: **40시행 내내 0** — 변한 적이 없다("표본 부족"과 구분해 적는다).
- 팔 단독은 두 세대 다 표본이 2~3개라 **판단을 보류**한다. 재측정이 필요하다.

### 거짓 성공은 셋이 아니라 넷이다

화면이 세운 것은 #14·#10·#17이지만 전량을 훑으면 **#3이 하나 더 있다**(통까지 195mm).
옛 파이프라인이 `DRIVE_PICK = [14, 10, 17, 13]` 넷만 골라 읽어 안 보였다.
원본 `trace.jsonl`로 리더를 거치지 않고 따로 세어 교차확인했다.

### 분모 문제 — 기록 안에서는 절대 안 보인다

`arm-v2`는 기록이 2건인데 시도는 3회다. 무대 배치 단계에서 끊긴 시행은 추적 파일에
한 줄도 안 남아서, **기록만 세면 성공률이 2/2 = 100%로 나온다.** 아는 만큼만
`project.py`의 `ATTEMPTS`에 출처와 함께 적고 화면에 "2 / 3 시도"로 표시한다.
근본 해법은 시행 스크립트가 **시도 자체를 기록하는 것**이다 — 4절 3번과 같은 뿌리다.

---

## 9. LeRobotDataset v3.0 내보내기

`core/lerobot_out.py`를 lerobot 환경 파이썬으로 → `~/capstone_tools/lerobot/<세대>/`.
**대시보드 빌드와 무관하다** — 학습 생태계로 나가는 별도 출구다.

### 스키마를 손으로 재현하지 않는다

v3.0 레이아웃(`meta/info.json` · `meta/stats.json` · `meta/tasks.parquet` ·
`meta/episodes/chunk-000/file-000.parquet` · `data/chunk-000/file-000.parquet` ·
`videos/<key>/chunk-000/file-000.mp4`)을 직접 만들면 그것도 결국 자체 구현이다.
**`lerobot` 라이브러리의 쓰기 API를 그대로 쓴다** — `LeRobotDataset.create` → `add_frame`
→ `save_episode` → `finalize`. 포맷이 구성상 맞고, lerobot이 v3.1로 가면 여기도 따라간다.

`~/lerobot`(소스 체크아웃, 0.6.1)이 `~/miniforge3/envs/lerobot`에 editable로 깔려 있다.
`mcap`·`mcap-ros2-support`를 그 환경에도 넣어 두었다. `torchcodec`은 없어 PyAV로 떨어지는데
읽고 쓰는 데 지장 없다.

### 싣는 키 — 전부 lerobot 예약 키

| 키 | 내용 |
|---|---|
| `observation.state` | 팔 5축 + 그리퍼 (rad) |
| `action` | **다음 시점의 관절 측정값** — 주의 아래 |
| `observation.environment_state` | 큐브 실좌표 xyz (Gazebo) |
| `observation.images.front` `.wrist` | MP4 (AV1) |
| `next.success` | **실좌표로 판정한 프레임별 성공** |
| `next.reward` | 위의 0/1 |

**`next.success`가 값어치다.** LeRobot v0.6.0이 `lerobot.rewards`로 성공 판정 모델
(Robometer·TOPReward 등)을 통합했는데, 그 모델들이 영상에서 *추정하려는* 값이 여기엔
시뮬레이터 실좌표로 들어 있다. 추정과 맞대 볼 정답이 데이터셋에 붙어 나간다.

**`action` 주의.** 이 기록에는 지령값이 없다 — 관절 *측정값*만 있다. 그래서 다음 시점의
측정값을 액션으로 쓴다(위치 제어 팔에서 흔한 변환). **지령을 기록한 척하지 않으려고**
docstring·데이터셋 카드(`README.md`)·화면 세 곳에 같은 문장을 적어 두었다.

### 사건 기반 기록을 고정 주기로 펴는 법

LeRobotDataset은 고정 fps를 요구하는데 이 기록은 단계가 바뀔 때만 남는 사건 기반이다.
**카메라 시각을 시간축으로 삼고** 관절·실좌표는 영차 유지(ZOH)로 채운다 — 없는 값을
보간해 만들지 않는다. 두 카메라를 **합쳐** 주기를 재는데, 번갈아 찍히므로 합집합이
촘촘하고 그 위에서 각 카메라를 ZOH하면 프레임이 겹칠 뿐 **버려지지 않는다**.
한쪽 주기에 맞추면 다른 쪽이 격자 사이로 빠진다.

주기·해상도는 **세대 안에서 하나**여야 한다. 시행마다 자연 주기가 3~4로 갈리므로
가장 빠른 쪽(4)에 맞춘다 — 느린 시행은 겹칠 뿐 잃지 않는다.
(처음엔 시행마다 따로 정했다가 A2·A3가 통째로 떨어져 나갔다.)

### 결과

| 세대 | 에피소드 | 프레임 | fps | 카메라 | 성공 프레임 |
|---|---|---|---|---|---|
| `arm-v1` | 3 | 1,625 | 4 | front · wrist | 98 |
| `arm-v2` | 2 | 187 | 1 | 없음 | 18 |
| `drive-v1` | 40 | 7,872 | 1 | 없음 | **0** |

- **카메라가 없는 세대도 내보낸다.** 저차원(관절·실좌표·판정)만으로도 LeRobotDataset은
  성립한다. 영상이 없다고 빠뜨리면 그 시행은 생태계 밖에 남는다
- 주행 72시행 중 32개는 실좌표가 없어 `observation.environment_state`를 채울 수 없다 —
  제외하고 **이유를 찍는다**(8절의 `drive_43` 절벽과 같은 자리)
- `live1.mcap`(163MB 실기록)은 세대 미상이라 제외

### 검증 — 내가 쓴 파일이 아니라 라이브러리가 읽는지

```
codebase_version v3.0 · fps 4 · 에피소드 3 · 프레임 1625 · 태스크 1
observation.images.front  (3, 162, 216)  dtype=torch.float32     ← MP4가 실제로 디코드된다
next.success=True 프레임 98 / 1625                                ← 44+27+27, 내보낸 값과 일치
data parquet (1625, 10) — 3에피소드가 파일 하나에                  ← v3.0의 묶기
meta/episodes/*.parquet: dataset_from_index · videos/.../from_timestamp ← 관계형 역인덱싱
```

v2가 에피소드당 파일 1개로 하다 파일시스템 한계에 부딪혀 v3.0에서 묶고 관계형 메타로
역인덱싱하도록 바뀐 그 구조가 그대로 나온다.

### 겪은 것

- **리더가 import 시점에 `sys.argv`를 읽고 있었다.** 모듈로 가져다 쓰는 쪽이
  자기 인자를 쓰면 남의 인자에 걸려 import가 그 자리에서 죽는다
  (`--epoch drive-v1` → `could not convert string to float: 'drive-v1'`). 기본값만 모듈에
  두고 argv 해석은 `main()`으로 옮겼다
- **`mcap_ros2`가 돌려주는 메시지 객체를 그대로 쌓지 말 것.** 필드마다 파이썬
  인스턴스가 달린 중첩 구조라 원본의 100배로 부푼다. `/tf`는 `robot_state_publisher`가
  관절 상태마다 전 링크를 재발행해 1000Hz를 넘기도 한다(SLAM 기록 실측 82,042건·1045Hz).
  **표본을 솎아서는 못 고친다** — `/tf`를 시간 기준으로 솎으면 판정이 깨진다(8mm → 240mm).
  줄일 것은 개수가 아니라 한 건의 크기다. `core/mcap_io.load`가 읽는 순간 9-튜플로 접는다
- **백그라운드 작업에는 상한을 걸 것.** 위 문제로 상한 없이 띄운 읽기가 RAM 25.6GB까지
  자랐고, `systemd-oomd`가 진범 대신 gnome-shell을 죽여 데스크톱이 무너졌다(2026-08-13).
  PSI 기준이라 150MB짜리 셸이 25.6GB짜리 python보다 먼저 죽는다.
  `systemd-run --user --scope -p MemoryMax=4G timeout 900 ...`
- **영상 인코더는 홀수 변을 싫어한다.** 녹화가 217×163이라 216×162로 맞춘다
