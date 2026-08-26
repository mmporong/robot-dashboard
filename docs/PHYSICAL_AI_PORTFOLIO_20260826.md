# Physical AI 검증 계층과 논문 적용안

- 기준일: 2026-08-26
- 상태: 현재 dashboard 기능을 Physical AI·sim-to-real 평가에 확장하는 추천안

## 이 저장소의 포트폴리오 역할

`robot-dashboard`는 정책이나 로봇을 직접 제어하지 않습니다. 실행을 다시 읽어 코드의 성공 주장과 물리 결과를 대조하고, 조건이 달라진 실행을 같은 성능으로 섞지 않도록 기록의 경계를 만듭니다.

이 역할은 Physical AI에서 중요합니다. 시뮬레이션과 실물 사이의 차이를 줄이려면 둘을 같은 스키마로 기록하고, 정책 버전·환경 조건·물리 결과를 한 실행 단위로 다시 비교할 수 있어야 하기 때문입니다.

## sim-to-real 기록 계약

각 MCAP 실행에 다음 메타데이터를 함께 둡니다.

| 범주 | 기록할 값 |
| --- | --- |
| 실행 | commit, 정책·판정기 버전, seed, 시작·종료 시각 |
| 환경 | `sim` 또는 `real`, 무대 세대, 로봇·센서 구성 |
| 시각 | 카메라 intrinsics·extrinsics, 조명·배경 조건, pointmap 좌표계 |
| 동역학 | 질량·마찰·지연·토크 제한·노이즈 범위 |
| 작업 | 초기 위치·회전·물체, ID·OOD 구분, 단계별 성공 조건 |
| 결과 | 코드 주장, 물리 정답, false positive·false negative, 안전 개입 |

시뮬레이션의 privileged state는 정답 생성에만 사용합니다. 배포할 성공 판정기는 카메라·관절·그리퍼처럼 실물에서도 얻을 수 있는 입력만 사용합니다.

## 논문을 dashboard에 옮기는 방식

| 논문 | 저장해야 할 증거 | 추천 |
| --- | --- | --- |
| [Domain Randomization](https://arxiv.org/abs/1703.06907) | 조명·재질·배경·카메라 조건과 각 조건의 검출·성공률을 연결합니다. | 시각 조건 필드 추가 추천 |
| [Dynamics Randomization](https://arxiv.org/abs/1710.06537) | 마찰·질량·지연·모터 강도 분포와 held-out 동역학 결과를 실행별로 남깁니다. | 물리 조건 필드 추가 추천 |
| [Closing the Sim-to-Real Loop](https://arxiv.org/abs/1810.05687) | 실물 rollout, 대응 시뮬레이션, 분포 갱신 전후를 하나의 계보로 묶습니다. | sim·real 비교 화면의 중심으로 추천 |
| [ACT](https://arxiv.org/abs/2304.13705) | chunk 길이, temporal ensemble 설정, 정책 지연, 단계별 성공률을 기록합니다. | SO-101 정책 비교에 추천 |
| [Diffusion Policy](https://arxiv.org/abs/2303.04137) | 예측·실행 horizon, denoising step, 추론 지연, 동일 초기 상태의 행동 다양성을 기록합니다. | ACT와 같은 평가표에 추천 |
| [RMA](https://arxiv.org/abs/2107.04034) | simulator privileged parameter, adaptation latent, 관측·행동 history 길이, 외란 후 회복 시간을 기록합니다. | Isaac Walk 확장에 추천 |
| [See like a Robot](https://arxiv.org/abs/2607.11498) | RGB·pointmap 입력, 카메라 위치, robot frame 정의, hand-eye 오차를 함께 남깁니다. | 시점 OOD 대조에 추천 |

논문 수치를 이 프로젝트의 결과처럼 옮기지 않습니다. 논문에서 가져오는 것은 실험 구조와 기록 항목이며, 성능 수치는 이 저장소가 읽은 실제 실행에서 다시 계산합니다.

## 추천하는 화면과 지표

### sim 대 real 비교

- 같은 Task Contract의 단계별 성공률
- 관절 추종·말단 위치·접촉 시점 차이
- 센서·DDS·정책 추론 지연
- 실물 실패가 randomization 분포에 반영된 이력

### 학생 판정기 비교

- 규칙 기준선, 트리 모델, GRU·TCN, 작은 causal Transformer
- 거짓 성공 탐지율, 오탐률, calibration error, 추론 지연
- 무대·세대 단위 분할 결과

Transformer가 단순 규칙이나 GRU보다 낫지 않으면 채택하지 않습니다. 이 저장소의 차별점은 모델 종류가 아니라 물리 정답과 재현 가능한 실행 기록입니다.

## 포트폴리오에 보여줄 것

1. `PICK_SUCCESS`였지만 실제로 통 밖이었던 실행을 한 장면으로 보여줍니다.
2. 같은 MCAP을 재생해 판정 규칙을 바꿔도 원시 기록이 유지되는 모습을 보여줍니다.
3. 시뮬레이션과 실물의 같은 필드를 나란히 놓습니다.
4. 논문 아이디어는 `적용 추천`, 실제 기능은 `구현·검증 완료`로 구분합니다.
5. 프로젝트 카드에서는 [jdamr_cube_ros](https://github.com/mmporong/jdamr_cube_ros), [gazebo-so101-capstone](https://github.com/mmporong/gazebo-so101-capstone)과 하나의 통합 시스템으로 연결합니다.
