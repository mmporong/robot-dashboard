#!/usr/bin/env python3
"""SO-101 팔로워 제어 GUI — 캘리브레이션·조그·IK 를 한 창에서.

CLI 세 단계(lerobot-calibrate → jog_test → ik_verify)를 버튼으로 옮긴 것이다.
캘리브레이션은 공식 `SOFollower.calibrate()` 와 **같은 절차·같은 파일 형식**으로
저장하므로(`~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json`)
나중에 lerobot CLI 도구를 써도 그대로 호환된다.

사용:
    conda activate lerobot
    python3 ~/so101_tools/arm_gui.py            # 기본 /dev/ttyACM0, id=follower

절차 (왼쪽부터 순서대로):
    ① 연결 → 관절 표에 현재 각도가 흐르면 통신 OK
    ② 캘리브레이션 — 처음 한 번만. 이미 파일이 있으면 이 단계는 건너뛴다
       [1] 토크 풀림 → 손으로 팔을 **가동범위 한가운데 자세**로 → [중립 기록]
       [2] [범위 기록 시작] → 각 관절을 끝에서 끝까지 손으로 움직임 → [기록 끝]
       [3] [저장] — 모터 EEPROM 과 JSON 에 함께 기록
    ③ 조그 — 관절별 ±버튼. **URDF +방향과 반대로 돌면 mapping.json 의 sign 을 -1 로**
    ④ IK — pan 축 기준 x·y·z[m] 입력 → [IK 이동]. 죠 끝을 자로 재서 검증

안전장치: 조그는 한 번에 5°, IK 이동은 3초 보간, 토크 OFF 버튼이 항상 살아 있다.
"""
import json
import math
import pathlib
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import arm_lib

ARM = arm_lib.JOINTS                      # 5관절 (kinematics 순서)
ALL = ARM + ['gripper']

# ── 안전 임계 (2026-08-19 발연 사고 후 도입) ──────────────────────────────────
# STS3215 의 기본 과온 한계는 70°C 다. 그보다 낮은 곳에서 스스로 멈춰야 보호가 된다.
TEMP_WARN, TEMP_STOP = 55, 62      # °C
TEMP_EVERY = 25                    # 폴링 몇 회마다 온도를 읽나 (매번은 비싸다)
STALL_GAP_DEG = 3.0                # 목표와 이만큼 벌어져 있으면 막힌 것으로 본다
STALL_MOVE_DEG = 0.4               # 0.5초 동안 이보다 덜 움직이면 '안 가고 있다'로 본다


# ── 하드웨어 워커 ────────────────────────────────────────────────────
class Worker(threading.Thread):
    """시리얼 통신 전담 스레드. UI 는 큐로 명령만 넣고 state 를 읽는다."""

    def __init__(self, port, robot_id):
        super().__init__(daemon=True)
        self.port, self.robot_id = port, robot_id
        self.cmd = queue.Queue()
        self.lock = threading.Lock()
        self.state = {'connected': False, 'calibrated': False, 'torque': False,
                      'recording': False, 'pos': {}, 'range': {}, 'log': [],
                      'speed_pct': 15}
        self.bus = None
        self._stop = False
        # 정지 신호 — HTTP 스레드가 큐를 우회해 직접 올린다. 워커가 3초 보간
        # 이동 중일 때 큐에 넣으면 이동이 끝나야 처리되므로 플래그로 끼어든다.
        self.abort = threading.Event()

    # -- UI 쪽 헬퍼 --
    def snapshot(self):
        with self.lock:
            return dict(self.state, pos=dict(self.state['pos']),
                        range={k: tuple(v) for k, v in self.state['range'].items()},
                        log=list(self.state['log']))

    def say(self, msg):
        msg = ' '.join(str(msg).split())        # 개행·중복 공백 제거
        if len(msg) > 140:
            msg = msg[:140] + '…'
        with self.lock:
            self.state['log'] = (self.state['log'] + [msg])[-8:]

    # -- lerobot 캘리브 파일 경로 (공식과 동일) --
    def calib_path(self):
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        return SO101Follower(SO101FollowerConfig(port=self.port, id=self.robot_id)
                             ).calibration_fpath

    def run(self):
        while not self._stop:
            try:
                cmd = self.cmd.get(timeout=0.25)
            except queue.Empty:
                self._poll()
                continue
            try:
                getattr(self, '_do_' + cmd[0])(*cmd[1:])
            except Exception as e:
                self.say(f'⚠ {cmd[0]}: {type(e).__name__}: {e}')

    # -- 명령들 --
    def _do_connect(self):
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
        norm = MotorNormMode.DEGREES
        motors = {j: Motor(i + 1, 'sts3215', norm) for i, j in enumerate(ARM)}
        motors['gripper'] = Motor(6, 'sts3215', MotorNormMode.RANGE_0_100)
        calib = {}
        p = self.calib_path()
        if p.exists():
            calib = {k: MotorCalibration(**v)
                     for k, v in json.loads(p.read_text()).items()}
        self.bus = FeetechMotorsBus(port=self.port, motors=motors,
                                    calibration=calib or None)
        try:
            self.bus._connect(handshake=False)
        except AttributeError:
            self.bus.connect(handshake=False)
        if calib:
            self.bus.write_calibration(calib)     # 파일 → 모터 EEPROM 동기화
        try:                                      # 속도 상한 확보 (위 _apply_motion_profile 주석 참조)
            self.bus.disable_torque()
            for m in ALL:
                self.bus.write('Maximum_Velocity_Limit', m, 254, normalize=False)
        except Exception as e:
            self.say(f'⚠ 속도 상한 설정 실패: {type(e).__name__} — 속도 제한이 안 걸립니다')
        with self.lock:
            self.state['connected'] = True
            self.state['calibrated'] = bool(calib)
        self.say(f'연결됨 · 캘리브 {"로드됨" if calib else "없음 — ② 진행 필요"}')

    def _do_disconnect(self):
        if self.bus:
            try:
                self.bus.disable_torque()
                self.bus.disconnect()
            except Exception:
                pass
        with self.lock:
            self.state['connected'] = False
            self.state['torque'] = False
        self.say('연결 해제')

    def _apply_motion_profile(self):
        """서보 레지스터에 안전 프로파일을 쓴다 — 어느 경로로 움직여도 적용된다.

        Goal_Velocity 는 이동 속도 상한[스텝/s], 0 이면 **무제한**이라 기본값 그대로
        두면 조그 한 번에도 팔이 튄다(실측 2026-08-14 "+5 눌렀는데 너무 세게").
        Acceleration(×100 스텝/s²)은 출발·정지 램프, Torque_Limit 는 막혔을 때
        밀어붙이는 힘의 상한이다 — 충돌해도 으스러뜨리지 않고 멈추게.
        """
        # 🔴 Goal_Velocity 는 Maximum_Velocity_Limit(주소 84, 1바이트) 이하일 때만
        # 반영된다. 초과하면 조용히 무시되고 서보가 최대 속도(≈40°/s)로 튄다 —
        # 공장 기본 상한이 65라 종전 매핑(5%→119, 100%→2000)은 전 구간이 무시됐다.
        # 상한을 254로 올려 두면 1 unit ≈ 0.087°/s 로 선형 제어된다(실측 2026-08-18:
        # vel 120→10.4°/s · 200→17.0°/s). 254 가 1바이트 최대라 상한 속도는 ≈22°/s.
        pct = self.snapshot()['speed_pct']
        vel = max(3, min(254, int(15 + pct / 100 * 239)))   # 1%→17(1.5°/s) · 100%→254(22°/s)
        self.bus.sync_write('Goal_Velocity', {m: vel for m in ALL}, normalize=False)
        self.bus.sync_write('Acceleration', {m: 15 for m in ALL}, normalize=False)
        self.bus.sync_write('Torque_Limit', {m: 600 for m in ALL}, normalize=False)

    def _do_speed(self, pct):
        pct = max(5, min(100, int(pct)))
        with self.lock:
            self.state['speed_pct'] = pct
        if self.snapshot()['connected']:
            self._apply_motion_profile()
        self.say(f'속도 {pct}%')

    def _do_torque(self, on):
        if on and self.snapshot()['recording']:
            self.say('⚠ 범위 기록 중엔 토크를 켤 수 없어요 — 손으로 움직이는 단계입니다')
            return
        if on:
            self._apply_motion_profile()          # 힘이 들어가기 전에 속도부터 묶는다
            # 한 서보씩 순차 인가 — 6개 동시 돌입 전류로 전원이 주저앉아 보드가
            # USB에서 떨어진 실측(2026-08-14 15:51)이 있다
            import time as _t
            for m in ALL:
                self.bus.enable_torque(m)
                _t.sleep(0.15)
        else:
            self.bus.disable_torque()
        with self.lock:
            self.state['torque'] = on
        self.say(f'토크 {"ON" if on else "OFF"}')

    def _do_neutral(self):
        import time as _t
        # 캘리브가 이미 있으면 실수 방지 이중 확인 — 10초 안에 두 번 눌러야 실행.
        # (실측 2026-08-14: 검증 중 실수로 눌러 모터 중립값이 덮였다. 파일이 있어
        #  재연결로 복구했지만, 한 번 누름으로 EEPROM을 덮는 건 위험하다)
        if self.snapshot()['calibrated']:
            last = getattr(self, '_neutral_arm', 0)
            self._neutral_arm = _t.time()
            if self._neutral_arm - last > 10:
                self.say('⚠ 이미 캘리브레이션이 있어요 — 정말 다시 하려면 '
                         '10초 안에 [기록]을 한 번 더 누르세요')
                return
        from lerobot.motors.feetech import OperatingMode
        self.bus.disable_torque()
        for m in ALL:
            self.bus.write('Operating_Mode', m, OperatingMode.POSITION.value)
        self._homing = self.bus.set_half_turn_homings()
        with self.lock:
            self.state['torque'] = False
            self.state['range'] = {}
        self.say('중립 기록 완료 → [범위 기록 시작] 후 관절을 끝까지 움직이세요')

    def _do_range(self, start):
        if start:
            self.bus.disable_torque()           # 손으로 움직이는 단계 — 토크 자동 해제
        with self.lock:
            self.state['recording'] = start
            if start:
                self.state['torque'] = False
                self.state['range'] = {}        # 이전 시도의 잔재를 비우고 새로 잰다
        self.say('범위 기록 중 (토크 자동 해제) — 각 관절을 손으로 끝에서 끝까지'
                 if start else '범위 기록 끝 → [저장]')

    def _do_save_calib(self):
        from lerobot.motors import MotorCalibration
        rng = self.snapshot()['range']
        if not rng:
            self.say('⚠ 범위 기록이 비어 있어요 — [2] 시작을 누른 **동안** 움직여야 '
                     '기록됩니다. 시작 → 관절 움직임 → 끝 → 저장 순서로')
            return
        missing = [m for m in ALL if m != 'wrist_roll'
                   and (m not in rng or rng[m][1] - rng[m][0] < 300)]
        if missing:
            self.say(f'⚠ 범위가 좁음: {missing} — [2]가 켜진 동안 그 관절을 끝까지 움직이세요')
            return
        # [1]에서 잰 값이 메모리에 없으면(서버 재시작 등) 모터 EEPROM에서 읽는다 —
        # set_half_turn_homings 가 이미 Homing_Offset 을 모터에 써 뒀다.
        homing = getattr(self, '_homing', None)
        if homing is None:
            homing = {m: self.bus.read('Homing_Offset', m, normalize=False)
                      for m in ALL}
            self.say('중립값을 모터 EEPROM에서 복원했어요')
        calib = {}
        for i, m in enumerate(ALL):
            lo, hi = (0, 4095) if m == 'wrist_roll' else rng[m]
            calib[m] = MotorCalibration(id=i + 1, drive_mode=0,
                                        homing_offset=homing[m],
                                        range_min=int(lo), range_max=int(hi))
        self.bus.write_calibration(calib)
        p = self.calib_path()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps({k: vars(v) for k, v in calib.items()},
                                indent=2))
        with self.lock:
            self.state['calibrated'] = True
        self.say(f'캘리브레이션 저장 완료 → {p.name}')

    def _do_jog(self, joint, delta):
        st = self.snapshot()
        if not st['calibrated']:
            self.say('⚠ 조그는 캘리브레이션(①~③) 후에 쓸 수 있어요')
            return
        if not st['torque']:
            self.say('⚠ 토크 ON 을 먼저 눌러 주세요')
            return
        cur = self.bus.sync_read('Present_Position', [joint])[joint]
        self.bus.sync_write('Goal_Position', {joint: cur + delta})
        self.say(f'{joint} {delta:+.0f}')

    def _do_stop_test(self, joint, target, wait_s):
        """이동을 걸고 wait_s 뒤 스스로 정지 — 워커 안에서 재므로 HTTP·폴링 지연이 없다."""
        if not (self.snapshot()['calibrated'] and self.snapshot()['torque']):
            self.say('⚠ 캘리브·토크 ON 후에 쓸 수 있어요')
            return
        rd = lambda: self.bus.sync_read('Present_Position', [joint])[joint]
        p0 = rd(); t0 = time.monotonic()
        self.bus.sync_write('Goal_Position', {joint: float(target)})
        time.sleep(wait_s)
        p1 = rd(); t1 = time.monotonic()
        self._do_stop()                                  # 실제 정지 경로 그대로
        t2 = time.monotonic()
        time.sleep(1.5)
        p2 = rd()
        self.say(f'속도 {(abs(p1-p0)/(t1-t0)):.1f}°/s · 정지호출 {1000*(t2-t1):.0f}ms · '
                 f'정지시 {p1:.1f}° → 최종 {p2:.1f}° (여유 {abs(p2-p1):.1f}°) · 목표였던 {target}°')

    def _do_grip_test(self, delta):
        """그리퍼 **하나만** 토크를 걸고 delta 만큼 움직인다 — 방향 확인용.

        팔 전체 토크를 켜면 늘어진 자세에서 돌입 부하가 크고(2026-08-14 USB 드롭),
        방향 확인에 필요한 것은 그리퍼 하나뿐이라 여기만 인가한다.
        """
        if not self.snapshot()['calibrated']:
            self.say('⚠ 캘리브레이션 후에 쓸 수 있어요')
            return
        self._apply_motion_profile()
        self.bus.enable_torque('gripper')
        before = self.bus.sync_read('Present_Position', ['gripper'])['gripper']
        self.bus.sync_write('Goal_Position', {'gripper': before + delta})
        time.sleep(2.0)
        after = self.bus.sync_read('Present_Position', ['gripper'])['gripper']
        self.say(f'그리퍼 {before:.1f} → {after:.1f} (명령 {delta:+.0f})')

    def _do_goto(self, joint, value):
        """관절 하나를 절대 목표로 — 슬라이더 조작용. 속도는 프로파일이 묶는다."""
        st = self.snapshot()
        if not st['calibrated']:
            self.say('⚠ 슬라이더는 캘리브레이션(①~③) 후에 쓸 수 있어요')
            return
        if not st['torque']:
            self.say('⚠ 토크 ON 을 먼저 눌러 주세요')
            return
        self.bus.sync_write('Goal_Position', {joint: float(value)})
        self.say(f'{joint} → {float(value):.0f}')

    def _do_stop(self):
        """그 자리에 정지 — 현재 위치를 목표로 다시 써서 붙든다 (토크 유지)."""
        self.abort.clear()
        if self.snapshot()['torque']:
            cur = self.bus.sync_read('Present_Position')
            self.bus.sync_write('Goal_Position', cur)
            # 남은 명령이 있어도 다음 이동이 기어가도록 속도를 바닥으로 내린다.
            # 속도는 [토크 ON]·[속도] 조작 때 프로파일이 다시 올린다.
            self.bus.sync_write('Goal_Velocity', {m: 8 for m in ALL}, normalize=False)
        self.say('⏹ 정지 — 현재 자세 유지')

    def _do_move_q(self, q_rad, seconds):
        """URDF 관절각[rad] 5개로 보간 이동."""
        st = self.snapshot()
        if not st['calibrated']:
            self.say('⚠ IK 이동은 캘리브레이션(①~③) 후에 쓸 수 있어요')
            return
        if not st['torque']:
            self.say('⚠ 토크 ON 을 먼저 눌러 주세요')
            return
        mapping = arm_lib.load_mapping()
        target = {j: mapping['signs'][j] * math.degrees(q_rad[i])
                  + mapping['offsets'][j] for i, j in enumerate(ARM)}
        cur = self.bus.sync_read('Present_Position', ARM)

        # ── wrist_roll 은 ±180 이 같은 자세다. 목표를 현재 위치에 가장 가까운 등가
        # 각도로 바꾸지 않으면, 0.26° 차이를 359.74° **역회전**으로 수행한다.
        #
        # 실측 2026-08-19: 현재 179.74° 에서 목표 -180.0° 을 그대로 보간했더니 팔이
        # 반대로 돌기 시작해 92.88° 에서 멈췄다(손목캠이 팔에 눌리는 경로). 전날
        # "wrist_roll → 180" 명령에 손목이 크게 돌아 카메라가 부서질 뻔한 것도 같은
        # 원인이다. mapping.json 의 offsets.wrist_roll = -180 을 넣은 뒤로는 IK 목표가
        # 항상 각도 경계에 놓이므로 이 보정 없이는 상시 발생한다.
        #
        # 등가 각도가 캘리브 범위(±180) 밖으로 나가면 되돌린다 — 그때는 먼 길이
        # 유일한 경로다.
        for j in ('wrist_roll',):
            d = target[j] - cur[j]
            if d > 180 and -180 <= target[j] - 360 <= 180:
                target[j] -= 360
            elif d < -180 and -180 <= target[j] + 360 <= 180:
                target[j] += 360
            moved = abs(target[j] - cur[j])
            if moved > 90:
                self.say(f'⚠ {j} 를 {moved:.0f}° 돌립니다 — 손목캠 간섭을 확인하세요')

        steps = max(2, int(seconds * 50))
        watch = None                              # 스톨 감지용 직전 위치
        for i in range(1, steps + 1):
            if self.abort.is_set():               # 정지 버튼 — 즉시 끊는다
                self._do_stop()
                return

            # ★ 이동 **중** 스톨 감지. 보간이 끝난 뒤에만 확인하면 그때까지는 막힌
            # 채로 계속 민다 — 서보가 타는 것은 그 구간이다(2026-08-19 사고).
            # 0.5초마다 보고, 위치가 안 변하는데 목표가 남아 있으면 즉시 끊는다.
            if i % 25 == 0:
                try:
                    now = self.bus.sync_read('Present_Position', ARM)
                except Exception:
                    self._do_stop(); self.say('⚠ 이동 중 읽기 실패 — 정지했습니다')
                    return
                if watch is not None:
                    moved = max(abs(now[j] - watch[j]) for j in ARM)
                    left = max(abs((now[j] - target[j] + 180) % 360 - 180) for j in ARM)
                    if moved < STALL_MOVE_DEG and left > STALL_GAP_DEG:
                        stuck = max(ARM, key=lambda j:
                                    abs((now[j] - target[j] + 180) % 360 - 180))
                        self._do_stop()
                        self.say(f'⛔ 스톨 감지 — {stuck} 가 {left:.1f}° 남기고 멈춰 '
                                 f'있습니다. 간섭을 확인하세요')
                        return
                watch = now
            a = i / steps
            s = a * a * (3 - 2 * a)
            self.bus.sync_write('Goal_Position',
                                {j: cur[j] + (target[j] - cur[j]) * s for j in ARM})
            time.sleep(0.02)

        # ★ 도달 검증. 못 갔으면 **그 자리에서 힘을 뺀다.**
        #
        # 2026-08-19 이 검증이 없어 wrist_flex(ID 4) 서보를 태웠다. 정합 스크립트가
        # 13개 지점 이동을 걸었고 전부 도달에 실패했는데, 목표가 그대로 남아 막힌
        # 방향으로 최대 전류를 계속 흘렸다. 스톨 상태가 장시간 유지돼 발연했고
        # 통신이 끊겨 소프트웨어로는 토크도 못 껐다.
        #
        # 실패를 로그로만 남기는 것과 안전한 상태로 되돌리는 것은 다른 일이다.
        # _do_stop() 이 현재 위치를 목표로 다시 써서 미는 힘을 없앤다.
        time.sleep(0.25)                      # 마지막 스텝이 반영될 시간
        try:
            fin = self.bus.sync_read('Present_Position', ARM)
        except Exception:
            self._do_stop(); self.say('⚠ 도달 확인 실패 — 정지했습니다'); return
        gap = max(abs((fin[j] - target[j] + 180) % 360 - 180) for j in ARM)
        if gap > STALL_GAP_DEG:
            worst = max(ARM, key=lambda j: abs((fin[j] - target[j] + 180) % 360 - 180))
            self._do_stop()
            self.say(f'⛔ 목표에서 {gap:.1f}° 남아 정지 ({worst}) — 간섭을 확인하세요')
        else:
            self.say('이동 완료 — 죠 끝을 자로 재서 기록하세요')

    # -- 폴링 --
    def _poll(self):
        if not (self.bus and self.snapshot()['connected']):
            return
        try:
            st = self.snapshot()
            if st['calibrated'] and not st['recording']:
                pos = self.bus.sync_read('Present_Position')
            else:
                pos = self.bus.sync_read('Present_Position', normalize=False)
            self._fail = 0
            with self.lock:
                self.state['pos'] = pos
                # 캘리브 전에는 **항상** 범위를 쌓는다. 범위는 "관절이 어디까지
                # 가봤나"라 더 쌓여서 손해 볼 일이 없고, [2]를 안 누르고 움직인
                # 수고가 날아가는 사고를 막는다(2026-08-14 실제로 그랬다).
                # 캘리브 후에는 pos 단위가 도(°)로 바뀌므로 섞지 않는다.
                if st['recording'] or not st['calibrated']:
                    for m, v in pos.items():
                        lo, hi = self.state['range'].get(m, (v, v))
                        self.state['range'][m] = (min(lo, v), max(hi, v))
        except Exception as e:
            # USB를 다시 꽂으면 /dev/ttyACM 번호가 바뀌어(ACM0↔ACM1 실측 3회)
            # 서버가 붙든 경로가 조용히 죽는다. 같은 실패가 이어지면 살아 있는
            # 포트를 다시 찾아 재연결한다 — 사용자가 서버를 재시작할 일이 없게.
            self._fail = getattr(self, '_fail', 0) + 1
            if self._fail == 1:
                self.say(f'⚠ 읽기 실패: {type(e).__name__}: {str(e)[:70]}')
            if self._fail >= 8:
                self._fail = 0
                # 재연결 전에 힘부터 뺀다. 통신이 끊긴 동안에도 서보는 마지막
                # 목표를 향해 계속 밀고 있다 — 막혀 있으면 그대로 탄다.
                try:
                    self.bus.disable_torque()
                    with self.lock:
                        self.state['torque'] = False
                    self.say('⚠ 통신 두절 — 토크를 내렸습니다')
                except Exception:
                    pass
                self._reconnect()
            return                    # 읽기가 실패한 회차엔 온도 감시를 건너뛴다

        # 온도 감시는 **위 try 밖**에서 부른다. 안에 두면 온도 읽기 실패가
        # 통신 두절로 오인돼 _fail 이 올라가고 엉뚱하게 재연결을 시도한다.
        try:
            self._guard(st)
        except Exception:
            pass

    def _guard(self, st):
        """과열을 감시해 임계를 넘으면 스스로 토크를 내린다.

        서보는 막히면 최대 전류를 흘려 몇 분 만에 위험 온도에 이른다. 사람이
        화면을 안 보고 있어도 멈추게 하려면 이 감시가 필요하다(2026-08-19 발연 사고).
        읽기가 비싸므로 매 폴링이 아니라 몇 초에 한 번만 본다.
        """
        if not st['torque']:
            return
        self._tick = getattr(self, '_tick', 0) + 1
        if self._tick % TEMP_EVERY:
            return
        temps = {}
        for m in ALL:
            try:
                temps[m] = self.bus.read('Present_Temperature', m, normalize=False)
            except Exception:
                pass
        if not temps:
            return
        with self.lock:
            self.state['temp'] = temps
        hot = {m: t for m, t in temps.items() if t >= TEMP_STOP}
        warm = {m: t for m, t in temps.items() if TEMP_WARN <= t < TEMP_STOP}
        if hot:
            try:
                self.bus.disable_torque()
            except Exception:
                pass
            with self.lock:
                self.state['torque'] = False
            self.say(f'🔥 과열 자동 정지 — {hot} (임계 {TEMP_STOP}°C). 전원을 확인하세요')
        elif warm and self._tick % (TEMP_EVERY * 4) == 0:
            self.say(f'⚠ 서보 온도 상승: {warm}')

    def _reconnect(self):
        import glob
        try:
            self.bus.disconnect()
        except Exception:
            pass
        cands = sorted(glob.glob('/dev/ttyACM*')) or sorted(glob.glob('/dev/ttyUSB*'))
        if not cands:
            self.say('⚠ 시리얼 포트가 없어요 — USB 케이블·보드 전원을 확인하세요')
            with self.lock:
                self.state['connected'] = False
            return
        old_port, self.port = self.port, cands[0]
        try:
            self._do_connect()
            self.say(f'자동 재연결 성공 {old_port} → {self.port}')
        except Exception as e2:
            with self.lock:
                self.state['connected'] = False
            self.say(f'⚠ 자동 재연결 실패: {type(e2).__name__}')

    def stop(self):
        self._stop = True


# ── UI ───────────────────────────────────────────────────────────────
def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--id', default='follower')
    a = ap.parse_args()

    w = Worker(a.port, a.id)
    w.start()

    root = tk.Tk()
    root.title(f'SO-101 팔로워 · {a.port}')
    root.geometry('980x560')

    # 상단 바
    top = ttk.Frame(root, padding=6)
    top.pack(fill='x')
    ttk.Label(top, text=f'포트 {a.port} · id {a.id}').pack(side='left')
    ttk.Button(top, text='연결', command=lambda: w.cmd.put(('connect',))
               ).pack(side='left', padx=4)
    ttk.Button(top, text='해제', command=lambda: w.cmd.put(('disconnect',))
               ).pack(side='left')
    ttk.Button(top, text='토크 ON', command=lambda: w.cmd.put(('torque', True))
               ).pack(side='left', padx=(16, 2))
    ttk.Button(top, text='토크 OFF', command=lambda: w.cmd.put(('torque', False))
               ).pack(side='left')
    status = ttk.Label(top, text='—')
    status.pack(side='right')

    body = ttk.Frame(root, padding=6)
    body.pack(fill='both', expand=True)

    # ① 관절 상태
    fL = ttk.LabelFrame(body, text='관절 상태', padding=6)
    fL.grid(row=0, column=0, sticky='ns', padx=4)
    pos_lbl, rng_lbl = {}, {}
    for r, m in enumerate(ALL):
        ttk.Label(fL, text=f'{r + 1} {m}').grid(row=r, column=0, sticky='w')
        pos_lbl[m] = ttk.Label(fL, text='—', width=9, anchor='e',
                               font=('monospace', 10))
        pos_lbl[m].grid(row=r, column=1)
        rng_lbl[m] = ttk.Label(fL, text='', width=13, anchor='e',
                               foreground='#888')
        rng_lbl[m].grid(row=r, column=2)

    # ② 캘리브레이션
    fC = ttk.LabelFrame(body, text='캘리브레이션 (처음 한 번)', padding=6)
    fC.grid(row=0, column=1, sticky='ns', padx=4)
    ttk.Label(fC, text='[1] 토크가 풀리면 팔을 손으로\n     가동범위 한가운데 자세로'
              ).pack(anchor='w')
    ttk.Button(fC, text='중립 기록 (토크 풀림)',
               command=lambda: w.cmd.put(('neutral',))).pack(fill='x', pady=2)
    ttk.Label(fC, text='[2] 각 관절을 끝에서 끝까지\n     (wrist_roll 은 안 해도 됨)'
              ).pack(anchor='w')
    ttk.Button(fC, text='범위 기록 시작',
               command=lambda: w.cmd.put(('range', True))).pack(fill='x', pady=2)
    ttk.Button(fC, text='범위 기록 끝',
               command=lambda: w.cmd.put(('range', False))).pack(fill='x', pady=2)
    ttk.Label(fC, text='[3]').pack(anchor='w')
    ttk.Button(fC, text='저장 (EEPROM + JSON)',
               command=lambda: w.cmd.put(('save_calib',))).pack(fill='x', pady=2)

    # ③ 조그
    fJ = ttk.LabelFrame(body, text='조그 (캘리브 후 · 토크 ON)', padding=6)
    fJ.grid(row=0, column=2, sticky='ns', padx=4)
    for r, m in enumerate(ALL):
        step = 10 if m == 'gripper' else 5
        ttk.Label(fJ, text=m).grid(row=r, column=0, sticky='w')
        ttk.Button(fJ, text=f'−{step}', width=4,
                   command=lambda m=m, s=step: w.cmd.put(('jog', m, -s))
                   ).grid(row=r, column=1, padx=1, pady=1)
        ttk.Button(fJ, text=f'+{step}', width=4,
                   command=lambda m=m, s=step: w.cmd.put(('jog', m, s))
                   ).grid(row=r, column=2, padx=1, pady=1)
    ttk.Label(fJ, text='반대로 돌면 mapping.json\n의 sign 을 -1 로',
              foreground='#888').grid(row=len(ALL), column=0, columnspan=3)

    # ④ IK
    fK = ttk.LabelFrame(body, text='IK 이동 (pan 축 기준, m)', padding=6)
    fK.grid(row=0, column=3, sticky='ns', padx=4)
    ent = {}
    for r, (k, v) in enumerate((('x', '0.20'), ('y', '0.00'),
                                ('z', '-0.05'), ('pitch°', '-90'))):
        ttk.Label(fK, text=k).grid(row=r, column=0, sticky='w')
        ent[k] = ttk.Entry(fK, width=8)
        ent[k].insert(0, v)
        ent[k].grid(row=r, column=1, pady=1)
    ik_out = ttk.Label(fK, text='', wraplength=180, foreground='#555')

    def ik_go():
        try:
            x, y, z = (float(ent[k].get()) for k in 'xyz')
            pitch = math.radians(float(ent['pitch°'].get()))
        except ValueError:
            ik_out.config(text='숫자를 확인하세요')
            return
        K = arm_lib.load_kinematics()
        bf = tuple(p + o for p, o in zip((x, y, z), arm_lib.PAN0))
        q = K.ik_best(*bf, pitch=pitch)
        if q is None:
            ik_out.config(text='IK 해 없음 — 리치/한계 밖')
            return
        fk = K.fk_pos(q)
        pan = tuple(p - o for p, o in zip(fk, arm_lib.PAN0))
        ik_out.config(text=f'q={[round(v, 3) for v in q]}\n'
                           f'예측 TCP ({pan[0]:+.3f}, {pan[1]:+.3f}, {pan[2]:+.3f})')
        w.cmd.put(('move_q', q, 3.0))

    ttk.Button(fK, text='IK 이동', command=ik_go).grid(
        row=4, column=0, columnspan=2, sticky='ew', pady=2)
    ttk.Button(fK, text='홈 자세',
               command=lambda: w.cmd.put(('move_q', [0.0, -0.3, 0.6, 0.5, 0.0], 2.5))
               ).grid(row=5, column=0, columnspan=2, sticky='ew')
    ik_out.grid(row=6, column=0, columnspan=2, pady=4)

    # 로그
    log = tk.Text(root, height=6, font=('monospace', 9), state='disabled')
    log.pack(fill='x', padx=6, pady=4)

    def tick():
        st = w.snapshot()
        status.config(text=('● 연결됨' if st['connected'] else '○ 미연결')
                      + (' · 캘리브 OK' if st['calibrated'] else ' · 캘리브 없음')
                      + (' · 토크 ON' if st['torque'] else '')
                      + (' · 기록 중' if st['recording'] else ''))
        unit = '°' if (st['calibrated'] and not st['recording']) else ' t'
        for m in ALL:
            v = st['pos'].get(m)
            pos_lbl[m].config(text='—' if v is None else f'{v:8.1f}{unit}')
            if m in st['range']:
                lo, hi = st['range'][m]
                rng_lbl[m].config(text=f'{lo:.0f}~{hi:.0f}')
        log.config(state='normal')
        log.delete('1.0', 'end')
        log.insert('end', '\n'.join(st['log']))
        log.config(state='disabled')
        root.after(200, tick)

    def close():
        w.cmd.put(('disconnect',))
        time.sleep(0.3)
        w.stop()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', close)
    tick()
    root.mainloop()


if __name__ == '__main__':
    main()
