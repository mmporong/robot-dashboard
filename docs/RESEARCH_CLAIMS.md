
## [blog] (url 없음)  2024-01-15
- Robot data production rates exceed site upload bandwidth by 2-3 orders of magnitude — Foxglove cites robots recording over 1GB/s while a site may have only 10–100 Mbps of upload — which makes selective/partial upload (specific segments or topics) an architectural requirement, not an optimization, for any camera-heavy telemetry product.
    ↳ quote=Robots may record over 1GB of data per second, but a site may have only 10 to 100 Mbps of upload bandwidth.
- Foxglove positions MCAP as the standard recording container for multimodal robotics data and the default storage format for ROS 2, with support for multiple message encodings (Protobuf, FlatBuffers, and more) — i.e. the ingest format a competing tool is expected to speak.
    ↳ quote=MCAP is the standard for recording multimodal robotics data
- Recommended file-splitting practice is time-based splitting with ~1 minute per file as a common standard, with the split interval reverse-engineered from a target file size, because upload reliability degrades sharply with file size (1GB trivial vs 500GB error-prone).
    ↳ quote=Uploading a 1GB file can be quite simple, but doing the same for a 500 GB file is much more cumbersome and error-prone
- Compression must be applied at the chunk level (or inside individual messages as JPEG/H.264) rather than wholesale with gzip, because whole-file compression destroys the MCAP index that enables efficient summary/partial extraction — a concrete constraint on how a run-adjudication backend can seek into recorded camera streams.
    ↳ quote=chunk compression ... preserve the index that lets you extract summary data efficiently
- Foxglove frames data retention as an explicit cost-driven policy decision, asserting that teams typically only access recorded data for a couple of weeks and that retention beyond a month is economically harmful — implying the paid value sits in short-horizon triage rather than long-term archival.
    ↳ quote=If team members are only accessing data for a couple weeks after it's been recorded, retaining it past a month is hurting your bottom line

## [primary] (url 없음)  2023-12-03
- Foxglove's edge agent uses a pull/on-demand upload model rather than streaming everything to the cloud: recordings sit on the robot and are only transferred when a human requests import from the Foxglove UI — a deliberate bandwidth-conservation architecture for fleets on metered/cellular links.
    ↳ quote=When a team member requests to import the file, your device will upload data to Foxglove in the background.
- The agent is explicitly designed as offline-first: it maintains robot↔cloud sync state across unstable or intermittent connectivity rather than assuming a stable link.
    ↳ quote=It even handles syncing state between your robot and the cloud in the presence of unstable or intermittent network connections.
- The agent's ingestion contract is a filesystem watch directory (default /srv/foxglove/agent/storage) over inotify that accepts MCAP or ROS bag files and deliberately skips files still being written — i.e., the edge interface is 'drop a completed file in a folder', not a custom SDK or live topic subscription.
    ↳ quote=The agent accepts "MCAP or ROS bag files" in the watched directory ... The agent "will ignore recordings in progress," requiring completed files to be moved into the directory.
- Edge data ingestion is a paid-tier feature at Foxglove — the Agent is gated to Pro and Enterprise plans, indicating the free/open Studio-style visualization is the funnel and the fleet data pipeline is where monetization occurs.
    ↳ quote=The Foxglove Agent can be used with Pro and Enterprise plans.
- The agent's deployment requirements are cloud-tethered and Linux-specific — a Debian-based distribution, an inotify-capable filesystem, and outbound HTTPS to api.foxglove.dev — so the stock edge path does not support air-gapped operation; a separate 'Edge Sites' product handles field store-and-forward.
    ↳ quote=Agents run on your robots and upload directly to the cloud, Edge Sites are optimized to store and forward from compute stationed in the field

## [blog] (url 없음)  ~2024-09 (article lists \"2 years ago\"; corresponds to the Rerun 0.18 release; author Nikolaus West)
- Rerun's 0.18 release replaced per-row indexing with column-chunk storage and measured a 100x improvement in write/ingestion speed and 35x lower memory overhead on a 2.25M-point scalar dataset (9 plots x 5 series x 50k f64) — evidence that naive row-per-message telemetry storage does not scale and that columnar chunking is the fix.
    ↳ quote=That's an improvement of 100x for write and ingestion speed, and 35x for memory overhead!
- A leading open-source robotics visualization tool only claimed support for recordings containing millions of time points starting with its 0.18 release (~Sept 2024) — i.e. handling large pre-recorded robot datasets was an unsolved engineering problem in this space until very recently, indicating a high performance bar for any new dashboard competing on visualization.
    ↳ quote=From Rerun 0.18 on, we therefore consider (potentially pre-recorded) datasets containing millions of time points supported
- Rerun's production architecture stores data as Apache Arrow column chunks keyed by entity path and component, and passes those chunks untouched from SDK through the data store to the visualizer where they are uploaded directly to the GPU — a zero-copy columnar pipeline is the reference stack for web/3D robot telemetry rendering.
    ↳ quote=Chunk columns are represented as Apache Arrow arrays, and the chunk header contains the chunk id, entity path, and other metadata
- Rerun exposes a dedicated columnar ingestion API (`send_columns`) for data that already exists in columnar form, bypassing the row-oriented time-context batcher — meaning batch/offline ingestion of pre-recorded runs is treated as a separate code path from live logging.
    ↳ quote=The `rr.send_columns` API is designed for cases where you already have your data in columnar form
- Rerun ships CLI-level recording maintenance (`rerun rrd compact` with `--max-rows`/`--max-bytes`) to merge files and re-pack chunks, showing that per-run file compaction/tuning is a required operational feature of a robot-data storage layer, not an optional extra.
    ↳ quote=Storage is based around chunks of component columns

## [primary] (url 없음)  2025-10-05
- 로봇 조작 정책 평가는 현재 통계적 보증 없이 소수 하드웨어 시행(대개 20~40회)의 경험적 성공률을 보고하는 것이 업계·학계 표준 관행이며, 논문은 이를 명시적 미해결 문제로 규정한다. 즉 '실행 판정·평가 레이어'는 아직 표준 도구가 정착되지 않은 영역이다.
    ↳ quote=Typically in practice, robot policies are often evaluated on a small number of hardware trials without any statistical assurances. ... most research studies report empirical success rates of policies evaluated on a small number (e.g., 20-40) of trials.
- 시뮬레이터만으로 산출한 성능 경계는 sim-to-real 갭(조명·텍스처·접촉 물리·마찰 계수 불일치) 때문에 편향되며, 시뮬 결과만으로는 실환경 결과에 대한 엄밀한 통계적 추론이 불가능하다. 시뮬 ground truth 기반 재판정은 '시뮬 내 판정의 정확도'는 올리지만 실세계 성능 주장으로 바로 승격되지 않는다.
    ↳ quote=the simulation-to-real gap precludes rigorous statistical inferences about real-world outcomes from simulation results alone ... performance bounds solely relying on large-scale simulation predictions can be biased
- 소수의 실-시뮬 페어 평가로 시뮬 편향을 보정하는 prediction-powered inference 방식(SureSim)은 동일한 성능 경계를 얻는 데 필요한 하드웨어 평가 노력을 20~25% 이상 절감한다 — 즉 시뮬 기반 평가 하네스의 가치는 '실기 시행 횟수 절감'이라는 정량 지표로 값이 매겨진다.
    ↳ quote=our approach saves over 20-25% of hardware evaluation effort to achieve similar bounds on policy performance
- 평가 대상 정책이 diffusion policy와 멀티태스크 파인튜닝된 π₀(VLA 파운데이션 모델)로 설정되어 있어, 모방학습·로봇 파운데이션 모델 평가 도구 수요가 2025년 시점에 학계 연구 주제로 이미 형성되어 있음을 보여준다.
    ↳ quote=Using physics-based simulation, we evaluate both diffusion policy and multi-task fine-tuned π₀ on a joint distribution of objects and initial conditions
- 실제 실험에서 실-시뮬 결과 상관계수는 페어 평가셋 평균 ρ≈0.59에 그친 반면 sim2sim 상관은 0.97로, 시뮬 판정이 실세계 성능의 대리 지표로서 갖는 신뢰도 한계가 정량적으로 드러난다(실험 규모: 페어 시행 n=60, 추가 시뮬 최대 N=700).
    ↳ quote=ρ is 0.59 on average on the paired evaluation set ... correlation of around 0.97

## [secondary] (url 없음)  2026-07-30
- NVIDIA는 정책 평가(policy evaluation) 자체를 별도 제품 라인으로 이미 세워 두었다 — Isaac Lab-Arena가 '대규모 GPU 가속 정책 평가'용으로 명시적으로 포지셔닝돼 있다. 즉 '조작 정책 평가 레이어'는 빈자리가 아니라 최상위 플레이어가 시뮬레이터·학습 스택과 묶어 선점 중인 영역이며, 시뮬 내 롤아웃 기반 평가로 정면 대결하는 전략은 NVIDIA 생태계와 직접 충돌한다.
    ↳ quote=Isaac GR00T for general-purpose humanoid development, and Isaac Lab-Arena for large-scale, GPU-accelerated policy evaluation.
- Lightwheel은 '평가·회귀 탐지'를 제품 구성요소로 분리해 판다: RoboFinals가 반복 가능한 대규모 병렬 롤아웃으로 정책을 테스트하고 실패를 원인 범주(행동 커버리지 부족·캘리브레이션 약함·학습 시나리오 공백)로 진단하며, RoboStack이 배포 후 롤아웃 결과·엣지케이스·실패를 되돌려 표적 데이터 수집을 촉발한다. 판정→실패 분류→재수집의 폐루프가 이미 상용 제품 형태로 존재한다.
    ↳ quote=RoboFinals tests policies across those scenes in repeatable, massively parallel rollouts, diagnosing failures as missing behavior coverage, weak calibration, or gaps in training scenarios. RoboStack deploys validated policies and returns rollout results, edge cases, and failures, triggering ta
- '평가를 CI에 배선한다'는 개념은 자율주행/차량 도메인에서 Applied Intuition에 의해 이미 상용화돼 있다 — 시나리오 라이브러리, 환경·센서 조건 변주, 병렬 시뮬레이션, CI 통합 평가가 한 세트로 팔린다. 로봇 조작(manipulation) CI의 빈자리 주장은 '개념이 없다'가 아니라 '이 패턴이 아직 매니퓰레이션으로 이식되지 않았다'로만 성립한다.
    ↳ quote=scenario libraries, varied environmental and sensor conditions, parallel simulation, and evaluation wired into continuous integration
- 데이터 공급 측은 이미 산업 규모로 굳어졌고(하루 1,000시간 이상 시연 데이터 수집, 2025년 15만 시간 이상 납품), Scale AI는 '데이터셋이 측정 가능한 모델 개선을 만들어내는지 검증'하는 내부 정책 파인튜닝까지 자사 파이프라인에 포함시켰다. 데이터 수집·라벨링 인접 영역에서 승부를 보려는 전략은 자본집약적 기존 사업자와 정면 충돌한다.
    ↳ quote=The company says its network collects more than 1,000 hours of demonstration data per day; it reported delivering over 150,000 hours of physical AI data during 2025.
- 로봇 학습 데이터의 교환 포맷 표준화는 LeRobotDataset v3.0(센서모터 시계열·행동·멀티카메라 영상·태스크 메타데이터)로 오픈소스 쪽에 수렴 중이다. 판정·평가 도구를 만든다면 자체 포맷이 아니라 이 스키마에 붙는 것이 채택 경로이며, 반대로 포맷 자체를 제품화하는 시도는 이미 무료 표준과 경쟁하게 된다.
    ↳ quote=LeRobotDataset v3.0 introduced a standardized format for multimodal robot-learning data

## [primary] (url 없음)  명시적 발행일 없음 — rerun-io/rerun 저장소 main 브랜치의 상시 갱신 문서 (2026-08-13 조회 기준)
- Rerun의 웹 3D 렌더링은 three.js 계열이 아니라 Rust `wgpu` 기반이며, 자체 고수준 렌더링 크레이트 `re_renderer`를 그 위에 올린다. 웹 빌드에서는 WebGPU가 가능하면 쓰고, 아니면 기능이 더 제한된 WebGL 에뮬레이션 레이어로 자동 폴백한다 — 즉 브라우저 3D 뷰어의 기능 상한이 실행 환경의 그래픽 API에 따라 달라진다.
    ↳ quote=The Rerun Viewer uses the [`wgpu`](https://github.com/gfx-rs/wgpu) graphics API. It provides a high-performance abstraction over Vulkan, Metal, D3D12, D3D11, OpenGLES, WebGL and [WebGPU]. ... On web builds, we use WebGPU when available on the Web, but automatically fall back to a WebGL based e
- Rerun 뷰어는 즉시 모드(immediate mode) 구조로 매 프레임 in-RAM 데이터 스토어를 쿼리해 렌더러에 먹이며, 쿼리 캐싱과 백그라운드 스레드 처리는 아직 '향후 계획' 상태다. 문서 스스로 이것이 '수백만 포인트' 급 대형 데이터셋을 보려면 필요한 작업이라고 명시한다 — 대용량 처리는 이미 해결된 문제가 아니다.
    ↳ quote=In the future we plan on caching queries and work submitted to the renderer so that we don't perform unnecessary work each frame. We also plan on doing larger operation in background threads. This will be necessary in order to support viewing large datasets, e.g. several million points.
- Rerun의 저장 계층은 Apache Arrow 기반의 인메모리(in-RAM) 시계열 데이터베이스 `re_chunk_store`다. 즉 조회 대상 데이터가 RAM에 올라가는 구조이며, 별도 서버측 시계열 DB나 디스크 기반 쿼리 엔진이 뷰어의 기본 경로가 아니다.
    ↳ quote=We use it [Apache Arrow] in our in-RAM data store, [`re_chunk_store`]. ... An in-memory time series database for Rerun log data, based on Apache Arrow.
- 네이티브 뷰어와 웹 뷰어가 동일한 Rust 코드베이스에서 나온다 — WebAssembly로 컴파일되어 얇은 `.html` + `.wasm` blob + 자동 생성 `.js` 브리지로 배포되며, egui/eframe이 네이티브와 웹 양쪽을 커버한다. 브라우저 배포에 별도 프런트엔드 스택을 두지 않는다.
    ↳ quote=can also be compiled to WebAssembly (Wasm) and run in a browser. ... a thin `.html`, a `.wasm` blob, and an auto-generated `.js` bridge
- 데이터 전송 경로는 디스크의 `.rrd` 파일 또는 gRPC(뷰어 또는 Rerun Server 대상)이며, MCAP/rosbag이 1차 포맷이 아니다. 게다가 `.rrd`는 완전한 하위/상위 호환을 아직 보장하지 않고 '직전 버전이 만든 파일까지만' 열 수 있다 — 장기 보관용 실행 기록 아카이브 포맷으로는 취약하다.
    ↳ quote=The logging data can be written to disk as `.rrd` files, or transmitted over gRPC to either a Rerun Viewer or a Rerun Server. ... `.rrd` files do not yet offer full backwards or forwards compatibility. However, the current version of Rerun will always be able to open `.rrd` files generated by 

## [secondary] (url 없음)  2026-04-16
- Antioch은 AI 로봇용 클라우드 시뮬레이션 소프트웨어로 2026년 4월 8.5M 시드를 A*·Category Ventures 주도로 조달했고, 이는 불과 4개월 전 4.5M 프리시드에 이은 것(총 ~13M) — '시뮬레이션 기반 로봇 테스트'가 독립 제품 카테고리로 VC 자금을 받고 있다는 증거이며, 사용자가 상정한 '판정·평가 레이어' 빈자리가 완전히 비어 있지는 않음을 뜻한다.
    ↳ quote=Antioch Inc., a developer of cloud-based simulation software for artificial intelligence-enabled robots, has raised $8.5 million in funding to accelerate the development of more autonomous systems outside the physical world. ... Today's round, which comes just four months after the startup rai
- Foxglove 공동창업자 Adrian Macneil이 Antioch에 개인 엔젤로 참여했다 — 시각화/데이터 플랫폼 진영의 당사자가 '테스트·평가 레이어'를 자사 제품이 덮는 영역이 아니라 별개의 인접 레이어로 본다는 신호. 시각화 정면 대결을 피하고 판정 레이어로 가라는 전략 가설을 지지하는 동시에, 그 자리가 이미 선점 경쟁 중임을 보여준다.
    ↳ quote=Others, including MaC Venture Capital, Abstract, BoxGroup and Icehouse Ventures also participated, as did angel investors including Shyam Sankar of Palantir Technologies Inc. and Adrian Macneil of Foxglove Inc.
- Antioch의 제품 축은 '기록된 실행의 재판정'이 아니라 '디지털 트윈을 대량 병렬로 돌려 배포 전 검증'이다 — 즉 경쟁자의 웨지는 시뮬레이션 실행 규모(throughput)이고, 실좌표 기반 성공/실패 재판정·상태기계 재구성 같은 판정 정밀도 축은 이 기사에서 다뤄지지 않는다.
    ↳ quote=create digital twins of any kind of robot, so they can quickly validate any AI-powered system before it's deployed in the real world ... they can carry out thousands of tests in parallel by spinning up multiple digital twins of their robots at once.
- 로봇 팀의 물리 검증 비용은 '창고 스테이징 수 주 + 테스트 시설에 수백만 달러' 규모로 주장되며, 이것이 이 카테고리의 지불 의사(willingness to pay) 근거로 제시된다 — 평가·테스트 도구의 예산 출처가 R&D 인프라 예산임을 시사.
    ↳ quote=Robotics teams spend weeks staging warehouses and investing millions into test facilities to validate their systems. ... The only economically viable path to reindustrialization runs through robotics and automation, and scalable testing is the rate-limiting step.
- Antioch은 자체 시뮬레이터가 아니라 진화하는 시뮬레이션 인프라를 개발자에게 연결하는 '플랫폼 레이어'로 스스로를 규정하며 AI 코딩의 Cursor에 비유한다 — 평가 도구의 경쟁 축이 엔진 소유가 아니라 오케스트레이션/워크플로 레이어에서 형성되고 있음을 보여준다.
    ↳ quote=The goal is to do the same for 'autonomy teams' by ensuring they're always able to access the most advanced simulation and infrastructure tools for testing robots.

## [primary] (url 없음)  2025-09-16
- 모방학습·로봇 파운데이션 모델 생태계의 사실상 표준 데이터 포맷(LeRobotDataset v3.0)은 ROS 계열의 MCAP/rosbag이 아니라 'Parquet(저차원 고주파 관절상태·액션) + MP4(카메라 프레임) + JSON 메타데이터'의 3층 구조를 채택한다. 즉 대시보드가 다루는 관절 시계열 + 동기 카메라 프레임 조합은 이 생태계에서 이미 표준화된 저장 스키마를 갖고 있으며, 자체 포맷을 만들기보다 이 스키마에 붙는 편이 정합성이 높다.
    ↳ quote=Low-dimensional, high-frequency data such as joint states, and actions are stored in efficient Apache Parquet files ... Frames are concatenated and encoded into MP4 files. Frames from the same episode are always grouped together into the same video ... A collection of JSON files which describe
- 에피소드(=시행) 1건당 파일 1개로 저장하는 설계는 수백만 에피소드 규모에서 파일시스템 한계에 부딪혀 실패했고, v3.0은 여러 에피소드를 한 파일에 묶은 뒤 관계형 메타데이터로 개별 에피소드를 역인덱싱하는 방식으로 바꿨다. 시행 단위 산출물을 파일 단위로 쌓는 대시보드 저장 설계는 규모가 커지면 동일한 벽에 부딪힌다.
    ↳ quote=In our previous `LeRobotDataset:v2` release, we stored one episode per file, hitting file-system limitations when scaling datasets to millions of episodes.
- 대용량 카메라 스트림의 대역폭·디스크 제약에 대한 이 생태계의 해법은 전량 다운로드가 아니라 스트리밍이다. v3.0은 `StreamingLeRobotDataset` 인터페이스로 디스크에 내려받거나 메모리에 적재하지 않고 허브에서 직접 스트리밍 접근하도록 네이티브 지원한다.
    ↳ quote=The new format also natively supports accessing datasets in streaming mode, allowing to process large datasets on the fly.
- 에피소드 단위 메타데이터는 단일 대용량 JSON이 아니라 청크된 Parquet으로 분리 저장되고, 별도로 피처별 집계 통계(mean/std/min/max)와 자연어 태스크→정수 인덱스 매핑이 관리된다. 즉 '시행 목록·필터링·집계' 레이어가 원시 데이터와 분리된 인덱스로 존재하며, 판정/회귀 추적 레이어는 이 메타데이터 층 위에 얹는 것이 자연스럽다.
    ↳ quote=Episode-level metadata is stored in chunked Parquet files rather than a single large JSON file ... Stores aggregated statistics (mean, std, min, max) for each feature ... Contains the mapping from natural language task descriptions to integer task indices
- 이 표준 스택의 범위는 '학습용 데이터 저장·적재'에서 끝난다 — PyTorch DataLoader 배치 결합과 `delta_timestamps` 기반 관측/액션 히스토리 윈도잉까지만 정의하고, 시행 성공/실패 재판정이나 정책 성공률 회귀 추적 같은 평가 레이어는 포맷·툴체인에 포함되어 있지 않다(글 전체에 ACT·diffusion policy·VLA 평가 워크플로 언급 없음). 판정·평가 레이어가 빈자리라는 가설을 뒷받침하는 근거.
    ↳ quote=By using LeRobotDataset with a PyTorch DataLoader one can automatically collate the individual sample dictionaries from the dataset into a single dictionary of batched tensors

## [primary] (url 없음)  날짜 미표기 (페이지 푸터 "Copyright 2026 Foxglove Technologies, Inc." — 2026-08-13 접근 시점 기준 최신 제품 페이지)
- Foxglove의 Primary Site는 컨트롤 플레인/데이터 플레인 분리로 3개 배포 모델을 제공하며, 온프렘 모델에서는 고객의 Kubernetes 클러스터에서 데이터 플레인을 돌리고 메시지 본문과 첨부는 고객 인프라에 남는다(Foxglove로는 디바이스 ID·타임스탬프·레코딩/토픽 이름 같은 인덱싱 메타데이터만 전송). 즉 '온프렘 지원'은 완전 자체 호스팅이 아니라 컨트롤 플레인은 여전히 Foxglove가 쥐는 하이브리드다.
    ↳ quote=Run Foxglove's data plane services in your own Kubernetes cluster in the cloud or on premises … Message contents and attachments stay in your infrastructure
- 완전 오프라인(에어갭) 배포는 표준 Enterprise 플랜에 포함되지 않고 별도 커스텀 Enterprise 계약을 요구한다 — 즉 방산·공장 등 에어갭 요구 고객에게는 카탈로그 가격이 아닌 개별 협상 영역이며, 이 지점이 상용 경쟁의 마찰 구간이다.
    ↳ quote=A "Fully Offline deployment" exists but requires "a custom Enterprise agreement" and is "not included in standard Enterprise plans."
- Foxglove는 대역폭·연결성 제약을 Edge Sites라는 별도 온프렘 스테이징 계층으로 푼다. 로봇 인근 물리적 근접 위치에 데이터를 먼저 쌓아두고, 클라우드(Primary Site)로의 업로드는 사용자가 선택적으로 큐잉해 올리는 pull/on-demand 구조다(창고·농지·차고 등 자원 제약 환경 명시).
    ↳ quote=safely store data on-premises before you decide to forward it to the cloud … mitigating the risks of unpredictable network connectivity or low bandwidth
- Foxglove 데이터 관리 제품의 포지셔닝은 '로봇 데이터의 단일 기록 시스템(system of record)' — MCAP/ROS/Protobuf/JSON/FlatBuffers를 통합 수집하고 디바이스·시간범위·토픽으로 자동 인덱싱해 검색·큐레이션하는 데이터 레이어에 머무른다. 제품 페이지 어디에도 실행 성공/실패 판정, 회귀 탐지, 평가 지표에 해당하는 기능 서술이 없다.
    ↳ quote=Ingest recordings from robots or local workflows and unify formats like MCAP, ROS, Protobuf, JSON, and FlatBuffers … Automatically index by device, time range, and topic
- Foxglove는 SOC 2 Type II 독립 감사를 완료했고 전송 구간 TLS 1.2, 저장 구간 AES 256-bit 암호화, SSO/SAML 인증, 계정 접근·구독·설정 변경 이벤트에 대한 감사 로깅을 갖췄다 — 로봇 개발도구 SaaS가 엔터프라이즈 계약에 진입하려면 넘어야 하는 최소 컴플라이언스 기준선이다.
    ↳ quote=Our security practices and policies have been independently verified in a SOC 2 Type II audit.

## [primary] (url 없음)  2025-03-31 (arXiv v1; v2 2025-04-02; CoRL 2025 채택)
- 실제 로봇 조작 정책의 '성공/실패 판정'은 이미 자동화 연구가 상용 수준 정확도에 도달했다 — AutoEval은 약 1000장(수집 10분 미만)의 성공/실패 이미지로 사전학습 VLM을 파인튜닝해 이진 성공 판정기를 만들고, 정확도 95% 이상인 판정기만 배포한다. 즉 '판정 레이어'의 핵심 기술은 좌표 기반 기하 판정이 아니라 학습된 시각 분류기 쪽으로 표준화되고 있다.
    ↳ quote=fine-tune a pre-trained vision-language model (VLM) for the task of binary success detection ... We use approximately 1000 images, which takes less than 10 minutes to collect ... choose to deploy success classifiers in AutoEval that have an accuracy of >95%
- 자동 판정 시스템의 신뢰성은 '사람이 직접 매긴 ground truth 평가와의 상관'으로 검증되며, AutoEval은 평균 피어슨 0.942, 정책 순위 왜곡 지표 MMRV 0.015를 보고했다. 판정·평가 제품을 팔려면 이 수준의 인간 대조 검증 수치를 제시해야 한다는 사실상의 기준선이 존재한다.
    ↳ quote=average Pearson score of 0.942 ... MMRV of 0.015 (plotted as 1−MMRV in Figure 7 of 0.985) ... an MMRV score close to zero indicates that it rarely disrupts the ranking of policies
- 로봇 조작 정책 평가의 인건비 병목은 정량적으로 확인된 실제 고통 지점이다 — OpenVLA(Kim et al. 2024) 결과 보고에는 수천 회 시행과 100시간 이상의 사람 노동이 들었고, AutoEval은 19시간 자율 평가를 사람 시간 3분으로(수동 대비 약 16시간) 줄여 감독 시간을 99% 이상 절감했다고 주장한다.
    ↳ quote=reporting results for Kim et al. 2024 required a few thousand evaluation trials and more than 100 hours of human labor ... 19 hours of real autonomous evaluation only costs 3 minutes of human time, compared to ≈16 hours if a human evaluator wanted to run the same number of trials ... AutoEval 
- 시뮬레이션 기반 평가(SIMPLER)는 신호는 잘 주지만 신뢰성이 떨어지고, 남은 sim-to-real 갭이 정책마다 다르게 작용해 순위를 왜곡한다 — Gazebo 실좌표 판정으로 '평가 레이어'를 파는 전략의 반대 근거이자, 실기체 평가와의 상관 검증 없이는 판정 결과가 팔리기 어렵다는 증거.
    ↳ quote=SIMPLER evaluations in simulation provide a better performance signal, but lack reliability ... different policies may suffer differently from the remaining sim-to-real gap in SIMPLER evaluations ... since evaluations are still run in the real world, there is no sim-to-real gap that could nega
- 이 영역은 학계가 무료·오픈소스 인프라로 이미 공급 중이다 — AutoEval은 WidowX 스테이션 2대를 대시보드로 개방하고, 정책 서버를 등록하면 성공률·영상 리포트를 W&B로 받으며, 코드는 오픈소스, 평가 데이터는 Hugging Face에 공개, 이용은 무료다(4개 태스크, 2026-01-01까지). 유료 전환 지점을 이 계층에서 잡기 어렵다는 신호.
    ↳ quote=open access to two AutoEval stations with WidowX robots ... The service is free to use. Code is open-source on GitHub; evaluation data is logged and publicly available on Hugging Face.

## [secondary] (url 없음)  2024-08-09
- READY Robotics — a vendor-agnostic robot software layer (ForgeOS) that raised at least $41.5M including strategic investment from Rockwell Automation — shut down in August 2024 when a funding round collapsed, demonstrating that a horizontal, hardware-neutral robotics software platform can fail commercially even with deep funding and a strategic industrial backer.
    ↳ quote=Multiple sources told The Robot Report a funding round fell through at the last minute, which caused the company to lay off its staff and close its doors. […] According to Crunchbase, READY Robotics raised at least $41.5 million
- The identified failure mechanism was that ForgeOS was an *additive* purchase rather than a replacement: customers still had to buy the robot vendor's controller and bundled software, then pay extra for READY's software plus an additional PC — a cost structure that never cleared the incremental-benefit bar. This is the direct analogue of a judgment/dashboard layer that sits on top of tooling teams already own.
    ↳ quote=A company has to buy the robot company's controller and robot that includes the software. But it also needs to buy READY's software and an additional PC, which leads to more expenses.
- Incumbent vendor ecosystems (training infrastructure, available programmers, established platforms) beat a challenger's ease-of-use advantage; the challenger needed anchor customers who would commit to its programming model, and could not find them. This is evidence against winning a robotics-tooling market on UX/capability superiority alone against entrenched ecosystems.
    ↳ quote=For READY to succeed, they needed to find big fish who bought into their programming language, and the two [aforementioned] problems prevented this.
- The buyer-side budget environment for robotics software was contracting at the time of the shutdown: North American industrial robot sales fell 30% in 2023 and continued to decline into 2024, indicating the failure was partly demand-side rather than purely product-specific.
    ↳ quote=[Industrial robot sales fell 30% in 2023] and [were down to start 2024]
- READY's shutdown was part of a cluster of robotics startup failures rather than an isolated case — Bossa Nova Robotics, Dextrous Robotics, Dorabot, and Small Robot Company also shut down in the same period, establishing a high sector-wide base rate of failure that any new robotics tooling venture must price in.
    ↳ quote=Other robotics startups that have recently shut down include Bossa Nova Robotics, Dextrous Robotics, Dorabot, and Small Robot Company.

## [secondary] (url 없음)  2025-11-12
- Foxglove는 2025년 11월 Bessemer Venture Partners 주도로 4천만 달러 시리즈 B를 유치했고, 2021년 창업 이후 누적 5,800만 달러 이상을 조달했다. 즉 로봇 데이터/시각화 레이어는 이미 자본이 두텁게 들어간 영역이며, 시각화 정면 대결의 비용 장벽이 매우 높다는 근거가 된다.
    ↳ quote=Foxglove, a San Francisco-based startup building a data and observability platform for robotics companies, has raised $40 million in Series B funding. The company has now raised more than $58 million since its 2021 founding.
- Foxglove는 스스로를 시각화 도구가 아니라 '모든 로보틱스 스타트업이 쓰는 데이터 스택 + ML 플랫폼'으로 규정하고 있다. 즉 이 회사의 로드맵은 단순 리플레이를 넘어 학습·평가 인프라 방향으로 확장 중이며, '판정·평가 레이어'가 무주공산이라는 가정을 위협하는 반증 근거다.
    ↳ quote="Our vision is to build the data stack and ML platform that every other robotics startup can use, so they don't have to reinvent the wheel," Macneil said. "We want to help every robotics company move faster."
- Foxglove의 고객 기반은 Amazon, Anduril, Chef Robotics, Dexterity, NVIDIA, Shield AI 및 다수의 자율주행·휴머노이드 기업으로, 매니퓰레이션(Dexterity, Chef Robotics)과 방산까지 포함한다. 신규 진입자가 노릴 매니퓰레이션 고객군에 이미 선점 관계가 형성되어 있다.
    ↳ quote=Foxglove's platform is used by customers such as Amazon, Anduril, Chef Robotics, Dexterity, NVIDIA, Shield AI, and a number of autonomous vehicle and humanoid robot companies.
- Shield AI는 Foxglove를 내부 도구로 쓰다가 자사 HiveMind 자율 스택에 임베드해 자사 고객이 쓰는 SDK의 일부로 만들었다. 개발자 도구가 아니라 타사 플랫폼의 '코어 인프라'로 재판매되는 임베드형 유통 경로가 실제로 작동한다는 사례다.
    ↳ quote=It initially used Foxglove internally but later embedded the tools into its HiveMind autonomy stack, making Foxglove part of the software development kit (SDK) that Shield's own customers use.
- Dexterity는 Foxglove 도입으로 개발 시간 20% 이상, 연간 15만 달러를 절감했다고 추정했다. 도입 이전에는 사내 자체 도구와 여러 로그 분석기에 의존했고 '기록·리플레이 불가'가 핵심 페인포인트였다. 즉 이 시장에서 유료 전환을 일으키는 검증된 ROI 서사는 '판정 정확도'가 아니라 '디버깅 시간 단축'이다.
    ↳ quote=Dexterity, a leading logistics robotics company, estimated that the Foxglove platform has saved it more than 20% in development time and $150,000 annually in tooling and development time. Before integrating Foxglove, Dexterity relied on in-house tools and various log analyzers.

## [blog] (url 없음)  2026-07-21
- Roboto shipped "Roboto Agents" (announced 2026-07-21 by CEO Benji Barash), an AI agent layer that automates log review, failure investigation, cross-fleet pattern discovery and event curation — i.e., an incumbent robotics-data vendor has already moved past visualization into automated failure analysis/triage, the adjacent space to "run adjudication."
    ↳ quote=Instead of manually analyzing robot data one run at a time, describe what you want to accomplish. Agents review logs, investigate failures, find patterns across your fleet, and curate the events that matter most.
- Roboto's own launch positioning explicitly declares replay/visualization tools (RViz class) structurally inadequate for fleet-scale work — a funded competitor publicly stating that head-on visualization is the wrong battleground, which corroborates the "don't fight Foxglove/Rerun on rendering" thesis but also means the escape route is already occupied.
    ↳ quote=Visualizers and replay tools like RViz were built for humans investigating one robot, one run at a time.
- Roboto claims the "learning loop" (continuous learning from every robot, run and failure) as its product category for Physical AI — meaning the evaluation/learning-loop framing is already staked out as marketing territory by an existing commercial player, which is counter-evidence to treating "judgment/evaluation layer" as an unclaimed gap.
    ↳ quote=Physical AI will not be built on models alone. It will be built on the ability to learn continuously from every robot, every run, and every failure.
- Roboto publishes a named production customer (BRINC, drones) attesting that agent-driven root-cause analysis cut complex failure diagnosis from hours/days to minutes and let non-expert staff diagnose problems — evidence that paying demand for automated failure adjudication exists, but demonstrated in aerial fleet operations rather than manipulation/pick-and-place.
    ↳ quote=Before, it could take hours or even days to root-cause a complex flight failure. Now, with Roboto's AI chat and agents, along with the ability to provide BRINC-specific context and code, we're identifying and solving edge cases in minutes. These features have also enabled employees less famili
- The launch page carries no pricing, no self-serve signup, and no mention of on-prem or air-gapped deployment; the sole conversion path is a sales-led demo booking — indicating Roboto runs an enterprise sales motion rather than published per-seat/per-robot list pricing.
    ↳ quote=Want that on your own logs? Book a demo and we'll run Roboto Agents on one of your robot's runs.

## [primary] (url 없음)  2026-07 (v0.4.0 릴리스, README 최신 뉴스 2026/07; 리더보드 재구축 2026/05)
- 조작(manipulation) 정책 평가용 통합 하네스가 이미 Apache 2.0 무료 오픈소스로 존재하며 규모가 크다 — AllenAI vla-evaluation-harness는 18개 시뮬 벤치마크 x 40여 개 VLA 모델 서버를 묶고, 부속 리더보드는 2,087편 논문에서 2,456개 모델 x 18개 벤치마크를 집계한다. 즉 '시뮬 벤치마크 성공률 평가' 자체를 유료 제품으로 파는 전략은 이미 무료 대체재와 정면 충돌한다.
    ↳ quote=**[Leaderboard](https://allenai.github.io/vla-evaluation-harness/leaderboard/)** | The largest unified VLA comparison: 2,456 models × 18 benchmarks, aggregated from 2,087 papers.
- 그럼에도 '회귀 테스트(regression testing)' 기능은 이 선도 오픈소스 하네스에서조차 아직 구현되지 않은 계획 단계 항목이다 — 아키텍처 문서의 Planned Features에 'Reference score comparison: Regression testing against known model+benchmark scores'가 남아 있다. 평가 실행은 해결됐지만 '실행 간 비교·회귀 탐지 레이어'는 여전히 빈자리라는 직접 증거.
    ↳ quote=Reference score comparison: Regression testing against known model+benchmark scores.
- 조작 정책 평가 분야는 팀마다 사설 평가 포크를 유지해 결과가 갈리고 버그 수정이 전파되지 않는 파편화 상태이며, 이 프로젝트의 존재 이유 자체가 '판정 프로토콜 표준화' 수요다. 판정·평가 레이어에 대한 수요가 실재한다는 근거.
    ↳ quote=In practice, every research team ends up maintaining private eval forks per benchmark. Results diverge. Bug fixes don't propagate. No one tests under real-time conditions where the environment keeps moving during inference.
- 리더보드의 점수는 대부분 논문 저자 자가보고(first-party)이고 벤치마크별 평가 프로토콜이 표준화되지 않아 직접 비교가 불가능하다고 스스로 경고한다 — 즉 '독립적 재판정(adjudication)'은 여전히 미해결. 하네스가 재현한 것은 40여 모델 중 일부(✓ 표시)뿐이다.
    ↳ quote=Evaluation protocols are not fully standardized across all benchmarks, so scores may not always be directly comparable.
- 이 하네스의 기록 기능은 v0.4.0(2026/07)부터 기본 활성화된 SQLite 에피소드 기록 + 선택적 비디오 캡처 수준이며, 아키텍처 문서 어디에도 ROS/rosbag/MCAP 언급이 없다. 즉 커버리지는 '시뮬 벤치마크 배치 평가'에 한정되고, ROS 2 실기·Gazebo 실행 기록의 실좌표 재판정·타임라인 동기 재생 영역은 비어 있다.
    ↳ quote=[2026/07] [v0.4.0] released. Recording on by default, pinned reproducible Docker rebuilds, DuoBench, and the LeRobot bridge below.

## [primary] (url 없음)  2026-07-07
- 오픈소스 LeRobot(Hugging Face)이 v0.6.0에서 조작 정책 평가를 단일 `lerobot-eval` CLI로 통합했다. 신규 6개(LIBERO-plus, RoboTwin 2.0, RoboCasa365, RoboCerebra, RoboMME, VLABench)를 포함해 총 9개 벤치마크 패밀리를 한 인터페이스로 묶었고, 각 벤치마크는 문서 페이지·Docker 이미지·CI에서 스모크 테스트되는 SmolVLA 베이스라인 체크포인트를 함께 제공한다. 즉 '조작 정책 평가 하네스 + CI 회귀 테스트'는 이미 무료 오픈소스로 존재하며 빠르게 성숙 중이라는 반증 증거다.
    ↳ quote=v0.5.0 planted the flag on LeRobot as an evaluation hub for VLAs; v0.6.0 makes it the real deal with six new simulation benchmarks, all runnable through the same lerobot-eval CLI, each with a docs page, a Docker image, and a SmolVLA baseline checkpoint smoke-tested in CI
- 성공 판정(success detection)과 진행도 추정이 로봇 학습 루프의 '빠진 조각'이라고 LeRobot 팀이 공식적으로 명시했고, v0.6.0에서 policies API를 본뜬 통합 reward models API(`lerobot.rewards`)를 신설해 HIL-SERL reward classifier, SARM, Robometer, TOPReward 4종을 하나의 인터페이스 뒤에 놓았다. '실행 판정(run adjudication)'이 실재하는 미해결 수요라는 직접 확인인 동시에, 그 자리를 오픈소스가 표준 API로 선점하기 시작했다는 신호다.
    ↳ quote=Success detection and progress estimation are missing pieces in the robot learning loop, and v0.6.0 gives them a home. LeRobot now has a unified reward models API (lerobot.rewards), mirroring the policies API, with four reward models behind one interface - the HIL-SERL reward classifier, SARM,
- Robometer는 100만 개 이상의 로봇 궤적으로 학습된 사전학습 범용 리워드 모델로, 원본 비디오와 언어 지시만으로 태스크 진행도와 성공 여부를 태스크별 추가 학습 없이 채점한다. 즉 판정 레이어의 핵심 기능이 '시뮬레이터 실좌표 없이, 카메라 영상만으로' 무료 사전학습 모델로 제공되기 시작했다 — 사용자 자산의 ground-truth 재판정 차별점이 시뮬 밖(실기)에서는 이 경로와 정면 경쟁하게 된다.
    ↳ quote=Robometer is a pretrained, general-purpose reward model: point lerobot/Robometer-4B at any LeRobot dataset and it scores task progress and success from raw video plus a language instruction, with no task-specific training required. It is built on Qwen3-VL-4B and trained via trajectory comparis
- TOPReward는 리워드 가중치 없이 완전 제로샷으로 동작해, 범용 VLM(Qwen3-VL)이 궤적 비디오와 태스크 지시를 받고 토큰 "True"의 로그 확률을 읽는 방식으로 성공을 판정한다. 두 리워드 모델 모두 데이터셋에 프레임별 진행도 곡선을 기록하는 라벨링 스크립트를 동봉해 데이터셋 품질 검사와 진행도 오버레이 영상까지 커버한다. 성공 판정 기능 자체가 이미 커모디티화 경로에 올라 있어, 판정 '알고리즘'만으로는 유료 제품의 해자가 되기 어렵다는 근거다.
    ↳ quote=TOPReward goes fully zero-shot: no reward weights at all. It wraps an off-the-shelf VLM (Qwen3-VL) and reads the log-probability of the token "True" given the trajectory video and the task instruction. Any capable VLM becomes a reward function. Both ship with labeling scripts that write per-fr
- 회귀·강건성 탐지가 벤치마크 설계 자체에 내장되고 있다 — LIBERO-plus는 조명·카메라 시점·지시문 재작성 등 7개 축으로 LIBERO를 약 10,000개 변형으로 확장해 '정책이 언제 깨지는지'를 알려주는 것을 목적으로 한다. 아울러 신규 벤치마크 연결용 'Adding a New Benchmark' 가이드로 플러그인 구조를 열었고, 병렬 평가 기본값을 async vectorized 환경으로 바꿔 최대 2배 속도를 확보했다. 상용 도구가 노릴 수 있는 '평가 실행 엔진' 자리는 이미 상당 부분 채워져 있음을 뜻한다.
    ↳ quote=LIBERO-plus stress-tests VLAs with roughly 10,000 perturbed variants of LIBERO across seven axes, from lighting and camera viewpoints to rewritten instructions. It tells you when a policy breaks.

## [primary] (url 없음)  2026-04-20
- Foxglove cut its entry paid tier price by roughly 84%: the plan (renamed from Team to Pro) now starts at $20/month including 1 TB storage and 3 Developer Seats, versus the previous $126/month for 3 seats — an effective drop from ~$42 to ~$6.67 per developer seat per month at entry level.
    ↳ quote=Pro now starts at $20/month and includes 1 TB storage plus 3 Developer Seats (previously $126/month for 3 seats, over $100/month savings!).
- Foxglove is de-emphasizing pure per-seat monetization: all paid plans now include unlimited free 'Basic Seats' (view-only/occasional users), with only 'Developer Seats' (users who author and edit layouts) counted as paid seats.
    ↳ quote=All paid plans now also include unlimited Basic Seats at no additional cost, so teams can share access more broadly.
- Foxglove's paid model is shifting from seat-count to data-volume metering — storage, query, indexing, and bandwidth are each priced as independent meters rather than bundled with seats.
    ↳ quote=Decoupled meters: Storage, query, indexing, and bandwidth are now priced independently.
- Foxglove applies automatic volume discounts per meter, so unit economics improve with data scale — meaning revenue comes from teams pushing large robot data volumes, not from headcount.
    ↳ quote=As your usage grows on each meter, your per-unit rate for that meter automatically drops.
- Foxglove frames the reduction as lowering the barrier to adoption for robotics teams (self-serve acquisition pressure), and the self-service pricing announcement makes no mention of self-hosting, on-premises, or air-gapped deployment — those remain outside the published self-service tiers.
    ↳ quote=to make it easier for robotics teams to get started and scale

## [primary] (url 없음)  게시일 미표기 (상시 갱신되는 벤더 가격 페이지). 푸터에 "© 2026 Foxglove Technologies, Inc." — 2026-08-13 확인 기준.
- Foxglove는 로봇 개발도구 SaaS 중 드물게 단가를 공개하며, 과금 축이 시트·디바이스·데이터량 세 갈래로 분리돼 있다. Pro는 기본 $20/월에 포함분(3 users, 5 devices, 1TB storage)을 넘기면 개발자 시트 $42/user/mo, 디바이스 $20/device/mo가 별도로 붙는다. 즉 '로봇 대수'와 '사람 수'가 각각 독립 과금되므로, 로봇 1대에 개발자 5명이 붙는 시뮬레이션·평가 워크플로에서는 시트 비용이 지배적이 된다.
    ↳ quote=Pro: "$20/month + additional usage" … "3 users included + $42/user/mo" … "5 devices included + $20/device/mo" … "1 TB storage included + additional usage"
- 온프렘·자체호스팅·BYO 스토리지·감사로그·SOC 2·커스텀 OIDC는 전부 Enterprise 전용이며 가격은 "Custom"(영업 협상)이다. 공개 단가 구간(Free/Pro)에서는 데이터가 Foxglove 클라우드의 US 또는 EU 리전에만 저장된다. 규정·에어갭 요구가 있는 산업 고객은 자동으로 세일즈 주도 계약 라인으로 밀려난다는 뜻이다.
    ↳ quote=Enterprise: "Custom" pricing with options for "Self-hosted data," "Forward-deployed engineers," … "On-premises deployment" … BYO Storage: "Index, search, and visualize data by connecting your existing AWS S3, Google Cloud Storage, or Azure Blob Storage bucket" (Enterprise only) … Enterprise: "
- 디바이스 과금 단위는 '월간 활성 디바이스'로, 해당 청구 주기에 연결되지 않은 디바이스는 과금하지 않는다. 시뮬레이션 기반 평가처럼 물리 로봇이 붙지 않는 사용 형태에서는 디바이스 축 매출이 발생하지 않아, Foxglove의 수익 구조가 '현장에 굴러가는 실기체 대수'에 묶여 있음을 보여준다.
    ↳ quote="Monthly Active Devices connected to the platform. Devices that are not connected or are inactive during the billing cycle are not charged."
- 무료 티어의 유료 전환 임계는 팀 규모·디바이스 수·데이터량에서 동시에 걸린다: 10GB 스토리지, 3 users, 5 devices, 1 project, 쿼리 1시간, 인덱싱·대역폭 100GB. 카메라 프레임 같은 대용량 스트림을 다루면 스토리지 10GB 한도에서 가장 먼저 막힌다.
    ↳ quote=Free Plan: "10 GB storage" · "1 hour" query time · "100 GB" indexing and bandwidth · "3 users" · "5 devices" · "1" project
- 학술 계정(.edu/.ac)은 $0에 1TB 스토리지·1TB 대역폭·10 devices·무제한 사용자를 받는다. 대학·연구실 채널을 무료로 장악해 두는 전략이며, 학계 발 로봇 파운데이션 모델·모방학습 평가 수요는 Foxglove 유료 매출로 잡히지 않는 공백 구간으로 남아 있다.
    ↳ quote=**Academic:** "$0" (for .edu/.ac email holders) … "1 TB storage" · "1 TB bandwidth" · "3 projects" · "10 devices" · "Unlimited users"

## [primary] (url 없음)  Undated FAQ; page carries \"Copyright © 2025 InOrbit, Inc.\" (retrieved 2026-08-13)
- InOrbit prices its SaaS by monthly active robots — specifically the high-water mark of daily unique active robots — not per seat or per data volume, with volume discounts and annual prepay for larger fleets and a free tier entry point.
    ↳ quote=you pay a variable monthly fee based on the number of monthly active robots (more specifically, the high water mark of daily unique active robots)
- InOrbit publishes a concrete list price for its Premium Support add-on: $3,000 per month on a 1-year commitment, one of the few openly stated dollar figures among robotics ops SaaS vendors.
    ↳ quote=$3,000 per month with a 1-year commitment
- InOrbit's stated scope is autonomous mobile robots (AMR fleets), not manipulation — its 'robot-agnostic' claim is explicitly bounded to AMRs, leaving pick-and-place run adjudication outside its product surface.
    ↳ quote=robot-agnostic and can control any autonomous mobile robots
- As of the page's 2025 copyright, InOrbit's documented ROS 2 support stops at Foxy and Humble, with no Jazzy listed — indicating the vendor tracks LTS distros used by fielded AMR fleets rather than current sim/manipulation stacks.
    ↳ quote=Foxy and Humble
- InOrbit is positioned as cloud-only SaaS with a deliberately thin edge agent (1GB RAM, 512MB disk, 1 shared CPU core) using low-bandwidth MQTTS/TLS 1.2+ and sampled telemetry (10-second regular, diff, event modes); the FAQ documents no on-premise or air-gapped deployment option.
    ↳ quote=1GB RAM, 512MB HDD, 1 CPU core (shared)

## [primary] (url 없음)  게시일 표기 없음 (2026-08-13 접속 기준 현행 가격표)
- Roboto의 가격은 시트당·로봇대수당이 아니라 데이터량(스토리지·대역폭)과 컴퓨트 크레딧 종량제로 매겨진다. Premium은 스토리지 1TB 초과분 $0.05/GB, 대역폭 250GB 초과분 $0.15/GB이며 컴퓨트는 월 250크레딧부터 시작해 추가 구매하는 구조다. 즉 로봇 대수가 아니라 '얼마나 많은 로그를 올리고 얼마나 돌리느냐'가 과금 축이다.
    ↳ quote=Premium: "1 TB, then $0.05 per GB. N/A if using own bucket" ... Data Bandwidth Premium: "250 GB included, then $0.15 per GB" ... Compute Premium: "Starting at 250 Compute Credits per month. Additional Credits can be purchased if needed"
- 온프렘/자체 호스팅과 데이터 리전 자유 선택은 Enterprise 전용 게이트다. Free/Premium은 데이터 리전이 "US & Europe"로 고정되고, "Self-managed deployment"와 "Data region: Any"는 Enterprise에서만 제공된다. 즉 에어갭·데이터 주권 요구는 상용 로봇 데이터 SaaS에서 최상위 계약으로만 충족되는 실재 수요다.
    ↳ quote=**Enterprise only** supports "Self-managed deployment" or bring-your-own-bucket arrangements. "Data region: Any" for Enterprise versus "US & Europe" for Free/Premium.
- SAML SSO와 감사로그(audit log)는 Enterprise 전용이며, RBAC는 무료 티어부터 전 티어에 포함된다. 즉 로봇 데이터 SaaS의 유료 전환 지점은 RBAC가 아니라 SAML SSO·감사로그·자체 배포 같은 기업 컴플라이언스 기능이다.
    ↳ quote=All tiers include "Role-based access control" and "Email authentication." Premium adds "Google SSO" and "Microsoft SSO." Enterprise includes all prior features plus "SAML SSO" and "Audit log" capabilities.
- 무료 티어는 보관기간 6개월·사용자 3명·스토리지 100GB·대역폭 25GB·컴퓨트 50크레딧(4 vCPU·8GiB 기준 50시간)으로 제한되며, 무제한 보관은 Premium/Enterprise의 "Custom" 항목이다. 즉 데이터 보관기간 자체가 유료화 레버로 쓰인다.
    ↳ quote=Free: "100 GB data storage" ... "25 GB" ... "50 Compute Credits in managed fleet (50 hours of 4 vCPU and 8 GiB memory)" ... "up to 3 users" ... Data retention: Free offers "6 months"; Premium and Enterprise allow "Custom" retention periods.
- Roboto는 자신을 "The analytics engine for Physical AI"로 포지셔닝하며 가격표의 기능 매트릭스는 데이터 관리·시각화(ROS/MCAP/ULG/이미지/비디오)·검색·처리(액션·트리거)·SDK/CLI/REST API로 구성된다. 성공/실패 재판정(run adjudication), 회귀 탐지, 성공률 리그레션 추적 같은 '평가 레이어'는 별도 과금 항목이나 기능 행으로 등장하지 않는다.
    ↳ quote="ROS, MCAP, ULG, Images, Videos, Console Logs, JSON, CSV & more" ... **Data Processing:** Compute credits; GPU support (Enterprise); concurrent actions; custom actions; active triggers; container images ... **API Access:** "SDK, CLI & REST API" ... Roboto is described as "The analytics engine 

## [primary] (url 없음)  No publish date stated; undated marketing homepage, accessed 2026-08-13. (A rendered 'Aug 12, 2026, 6:51 PM UTC' string appears to be a dynamic render/fetch timestamp, not a publication date.)
- Formant has repositioned away from 'robot fleet management / observability tooling' toward an enterprise governance framing — 'the operating layer for physical AI' — and the site now lives at formant.ai rather than the older formant.io. This is a pivot in go-to-market language toward enterprise/AI-deployment buyers, not developer tooling.
    ↳ quote="The operating layer for physical AI" — "the governed layer through which physical AI reaches enterprise reality, beneath whatever machines and models win."
- Formant's stated platform scope is monitoring, intervention, evidence, and orchestration for robots/operators/enterprise systems/AI agents. 'Evidence' is a first-class function, but it is framed as operational/enterprise audit evidence for live deployments — the homepage contains no mention of ROS, rosbag/MCAP, simulation, CI, regression testing, success-rate tracking, or manipulation. This supports the 'run adjudication / evaluation as a product' gap being unclaimed by Formant.
    ↳ quote="Formant connects robots, operators, enterprise systems, and AI agents through a governed layer for monitoring, intervention, evidence, and orchestration."
- Formant publishes no pricing of any kind — no per-seat, per-robot, or per-data-volume tiers, no free tier — and routes all interest through a single sales-led CTA. Robotics ops SaaS at this tier is enterprise contract-negotiated, not self-serve.
    ↳ quote="START A DISCOVERY SESSION" (the only call-to-action on the page; appears twice, with no pricing page or figures anywhere on the homepage)
- Formant's revenue model bundles professional/deployment services with the software rather than selling software alone, indicating that pure self-serve SaaS has not been sufficient in this market and that deployment engineering is part of what customers buy.
    ↳ quote="Software and services, run like a boot camp."
- Formant's named customers are industrial and field-robot fleet operators (BP, Burro, Hullbot, Cala) operating production fleets — not manipulation research labs or ML evaluation teams. The buyer persona is fleet operations, which is a different budget owner than a manipulation run-adjudication/evaluation tool would target.
    ↳ quote="TRUSTED BY INDUSTRY LEADERS" ... "Real deployments. Not pilots." (case studies: BP, Burro, Hullbot, Cala)

## [primary] (url 없음)  페이지에 게시일·최종수정일·저작권 연도 표기 없음 (no date metadata on page); 접속일 2026-08-13
- SVRC의 'Fearless' 플랫폼은 정책 평가(policy evaluation) 실행 횟수 자체를 과금·쿼터 단위로 삼는다 — 무료 Academic 티어에 '월 100 runs' 상한이 명시되어 있어, '실행 판정·평가'가 이미 상용 SKU의 미터링 축으로 상품화되어 있음을 보여준다.
    ↳ quote=Policy Evaluation (100 runs/month)
- 이 플랫폼은 ROS Bag·LeRobot Parquet·RLDS·HDF5를 수집하고 '관절과 카메라가 동기화된 프레임 단위 리플레이'를 제공한다 — 즉 '타임라인 동기 카메라 + 관절 되감기'라는 현 자산의 핵심 기능은 이미 상용 제품에 탑재되어 있으며, 시각화·리플레이 자체로는 차별화가 되지 않는다는 반대 근거다.
    ↳ quote=Frame-level replay with synced joints and cameras
- 가격 모델은 순수 시트당·로봇당 종량제가 아니라 '정액 + 다축 상한' 구조다: Pro는 월 $249에 팀원 20명, 스토리지 5TB, 함대 로봇 10대, API 1,000 req/min으로 캡을 걸고, 초과 수요는 Enterprise 커스텀 가격으로 넘긴다.
    ↳ quote=$249/mo — "Up to 20 team members", "5 TB storage", "Fleet Integration (up to 10 robots)", "API access (1,000 req/min)"
- 온프렘·에어갭은 무료·Pro가 아니라 Enterprise 전용 게이트로 배치되어 있고, 배포 형태는 Docker+Kubernetes(Helm), 대상은 '방위·규제 산업'으로 명시된다. SOC 2는 Type II 감사 '진행 중' 단계이며 GDPR·미/EU 데이터 레지던시를 함께 내건다.
    ↳ quote=Self-Hosted: "Docker + Kubernetes (Helm)" / "Air-gapped for defense/regulated" / "SOC 2 Type II audit in progress"
- 유료 전환 지점이 시각화가 아니라 '실패 분석·회귀·A/B 비교' 레이어에 걸려 있다 — 'Failure Mining & Retraining Pipeline'은 Pro 이상 기능이고, 이상 에피소드 자동 플래깅·근본원인 클러스터링과 정책 버전 A/B 벤치마크를 판매 포인트로 내세우며, 대상 정책군으로 ACT·Diffusion Policy·Octo·RT-2·OpenVLA를 명시해 모방학습/VLA 평가 수요를 직접 겨냥한다.
    ↳ quote="Failure Mining & Retraining Pipeline" (Pro tier and above) — "Auto-flag anomalous episodes and cluster by root cause"; "Benchmark cycle times and A/B test policy versions"; supports "ACT, Diffusion Policy, Octo, RT-2, OpenVLA"

## [primary] (url 없음)  2026-02-26 (arXiv v1, cs.RO 2602.22818; ICLR 2026 게재 논문)
- 로봇 학습 분야의 재현성 저해 요인으로 '평가 파이프라인(evaluation pipelines)'이 알고리즘·데이터 처리와 나란히 명시적으로 지목되며, 로봇공학에서는 하드웨어 편차가 이를 가중시킨다고 지적한다. 즉 '실행 판정·평가 레이어'가 미해결 문제라는 주장을 지배적 오픈소스 라이브러리 저자들이 직접 인정한 근거다.
    ↳ quote=The deep learning literature has consistently demonstrated that minor implementation differences in algorithms, data handling, and evaluation pipelines can lead to significant variance in results (Henderson et al., 2018). In robotics, these issues are compounded by hardware variability, furthe
- 반증 근거: 시뮬레이션 기반 매니퓰레이션 평가 하네스(LIBERO, Meta-World)와 '테스트 에피소드 N회에 대한 성공률 보고' 프로토콜은 이미 LeRobot API에 무료로 네이티브 통합되어 있다. 시뮬 평가 하네스+성공률 집계는 상용화 여지가 좁은 이미 채워진 슬롯이다.
    ↳ quote=we provide evaluation support via the lerobot API for both LIBERO (Liu et al., 2023) and Meta-World (Yu et al., 2020), two popular simulation environments that are often used as benchmarks for robot learning research ... Typical LIBERO evaluation protocols report the success rate over a number
- Hugging Face는 시뮬레이션이 접촉이 많은(contact-rich) 복잡한 조작 과제에는 부적합하다고 판단해 실세계 데이터 학습을 원칙으로 삼고, 시뮬레이션은 '알고리즘의 체계적 평가' 용도로만 남겨두었다. 이는 Gazebo pick-and-place 실좌표 판정 자산의 포지션을 양분한다 — 평가 용도라는 방향은 맞으나, 정책 학습 데이터로서의 시뮬 가치는 업계 리더가 스스로 낮게 본다.
    ↳ quote=In practice, simulation proves challenging for the kind of contact-rich, complex tasks lerobot targets. This justifies the library's choice to train as much as possible on real-world data, relying on simulation primarily for the systematic evaluation of robot learning algorithms.
- 로봇 학습 데이터의 사실상 표준 포맷은 MCAP/rosbag이 아니라 LeRobotDataset(.parquet 테이블 + .mp4 압축 영상 + 경량 메타데이터, torchcodec 온더플라이 디코딩, HF 허브 스트리밍)이다. 논문은 ROS bag을 '통합을 가로막는 파편화된 포맷' 중 하나로 분류한다.
    ↳ quote=Data is often released in varied formats like TensorFlow Datasets, ROS bags, or bespoke JSON layouts. The absence of a universal, modality-rich schema prevents the seamless aggregation of disparate datasets into larger mixtures. ... The LeRobotDataset format partitions robotic data into tabula
- 측정된 추론 지연 데이터: ACT(52M)는 RTX 4090/A100에서 약 100~200Hz를 내는 반면 π0(3.5B)는 저사양 기기에서 5초 하드 타임아웃 내 추론을 완료하지 못한다. 로봇 파운데이션 모델의 엣지 배포가 실측으로 병목임을 보이며, 원격 추론 서버-로봇 클라이언트 분리 아키텍처의 필요 근거가 된다.
    ↳ quote=larger models such as π0 require substantially longer per each forward passes on average on all platforms, and even fail to complete inference within the 5s limit on lower-tier devices, underscoring the challenges in deploying robotics foundation models in practice.

## [primary] (url 없음)  2026-04-21
- Foxglove repositioned itself in April 2026 from a visualization tool to an "observability and data platform for Physical AI," launching Data Search and Curation — direct querying of multimodal robotics data plus tagging/annotation to build training and validation datasets — meaning the incumbent is now competing on the data/curation layer, not just the viewer.
    ↳ quote=Foxglove, a leading observability and data platform for Physical AI, announced Data Search and Curation, a new set of capabilities that helps robotics teams replace fragmented, manual data workflows with a unified platform to find and curate the mission-critical events, anomalies, and system b
- Foxglove introduced a free "Basic Seat" tier for visualization, explicitly to extend access beyond engineering to QA, triage, safety review, and management — confirming a seat-based pricing model and signaling that pure viewer/visualization functionality is being commoditized to zero price.
    ↳ quote=The new free Basic Seat tier makes it easier for robotics organizations to extend access beyond engineering to teams involved in QA, triage, safety review, and management, giving more stakeholders direct visibility into robot behavior, system performance, and operational events.
- Foxglove's answer to on-prem/data-residency demand is "Bring Your Own Storage" (BYOS): a self-hosted data lake where the customer keeps data at rest in their own storage while Foxglove still runs the managed database/compute — i.e. partial, not air-gapped, deployment.
    ↳ quote=The company also expanded the Foxglove Data Platform with Bring Your Own Storage (BYOS), a new self-hosted data lake deployment model allowing customers to maintain full control over data at rest while still providing the benefits of a fully managed database
- Foxglove's stated managed-compute scope in the BYOS model includes evaluation alongside indexing, query, search, and metadata workflows — evidence that the largest incumbent is already extending toward the evaluation layer rather than leaving it vacant.
    ↳ quote=a self-hosted deployment allowing customers to maintain data in their own cloud storage while Foxglove manages indexing, query, search, evaluation, and metadata workflows
- Foxglove frames the core customer pain as data triage/selection at scale (finding the ~1% of data worth acting on), not as verdict correctness or re-adjudication of run outcomes — leaving ground-truth-based success/failure re-judgment outside its stated value proposition.
    ↳ quote=Robotics teams are generating more data than ever, but the real challenge is finding the critical 1% that drives improvement in the real world.

<!-- 총 115개 -->
