# SO-101 MuJoCo 미러 (2026-08-20)

실팔 패널 서버(8765)의 `/state`를 10Hz로 읽어 MuJoCo 뷰어에 그대로 비추고,
뎁스캠 `/blob`이 보는 빨간 물체를 체스말 프록시(빨간 원기둥 Ø22×70mm)로
배치한다. **읽기 전용 — 팔로 나가는 명령은 일절 없다.**

## 좌표 검증 (frame_fit.py)
- MJCF(`so101_new_calib.xml`, onshape-to-robot)와 실팔 K(kinematics.py)는
  같은 URDF 출신 — `qpos = URDF q 직결`, K의 TCP = 모델 `gripperframe` 사이트.
- 작업영역 28자세 Kabsch 적합 **RMS 0.00mm**, 변환은 순수 평행이동
  `p_sim = p_K + (-0.02, 0, 0.05)` → `sim_frame.json`.

## 사용 (rlwalk 환경의 mujoco 3.11)
```bash
cd ~/so101_tools/sim
# 실시간 미러 (서버 8765 가동 중일 때)
~/miniforge3/envs/rlwalk/bin/python sim_view.py
# 정지 자세 + 물체 수동 배치
~/miniforge3/envs/rlwalk/bin/python sim_view.py \
  --deg "shoulder_pan=-6.3,shoulder_lift=-2.2,elbow_flex=0.9,wrist_flex=88.1,wrist_roll=0,gripper=2.6" \
  --piece-at "0.19,0.02" --piece lying
# 무화면 스냅샷 (--cam wrist_cam 이면 손목캠 시점)
... sim_view.py --deg "..." --piece-at "0.19,0.02" --snapshot out.png
```

## 새 기기 세팅
메시는 강의 자료를 심링크로 쓴다 (리포에는 미포함):
```bash
ln -sfn ~/so101_imitation_learning/106_so101_MUJOCO_imitation_learning/103_robot_pick_n_place/meshes ~/so101_tools/sim/meshes
```
좌표 재적합이 필요하면(모델 교체 등):
```bash
python3 ~/so101_tools/sim/gen_ref_poses.py            # 시스템 파이썬 (K 필요)
~/miniforge3/envs/rlwalk/bin/python ~/so101_tools/sim/frame_fit.py
```
