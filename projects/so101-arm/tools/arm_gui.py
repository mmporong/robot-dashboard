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
STALL_LAG_DEG = 15.0               # 움직이고는 있어도 보간 목표에서 이만큼 뒤처지면 끊는다
                                   # — 마찰로 기어가는 부분 스톨(creeping stall)도 서보를 태운다
ROLL_FIRST_DEG = 20                # wrist_roll 이 이보다 크게 바뀌면 회전을 먼저 끝낸다
LIMIT_MARGIN_DEG = 2.0             # 캘리브 범위 끝에서 이만큼은 남기고 멈춘다
TORQUE_ON_TOL_DEG = 3.0            # 토크 켜기 전 검사의 **바깥** 허용 — raw 카운트로 환산해 쓴다
STOP_TEST_MAX_S = 3.0              # stop_test 대기 상한 — 워커가 오래 자면 감시가 전부 멎는다

# 전류는 **가장 빠른 스톨 신호**다. 온도는 후행 지표(이미 뜨거워진 뒤 올라간다)이고
# 위치 기반 감지도 0.5초를 기다려야 한다.
#
# STS3215 데이터시트 실사양 — 단위는 6.5mA/LSB.
#   7.4V 19kg (리더): 무부하 150mA · 정격 650mA · 스톨 2.5A
#   12V  30kg (팔로워, **이 팔**): 무부하 180mA · 정격 900mA · 스톨 2.7A
#                                   입력 4~14V · Kt 11kg.cm/A
# ★ 리더와 팔로워는 다른 모델이다. 리더(7.4V)에 12V 를 꽂으면 모터가 탄다.
# 스톨 값(≈415)에 임계를 두면 이미 늦다. 정격(≈138)의 두 배쯤에서 끊는다.
CURRENT_STOP = 250                 # ≈1.6A (12V 정격 900mA 의 1.8배). 실측으로 조정할 것
CURRENT_HOLD = 2                   # 순간 피크(가속·정지)로 오작동하지 않도록 연속 확인

# 서보 자체 보호 (연결 시 EEPROM 에 써 둔다).
#
# ★ 공장 기본값은 Protection_Current=0(과전류 보호 꺼짐), Unloading_Condition=0
# (토크 해제 조건 없음)이다. 즉 **서보가 스스로를 지킬 장치가 비활성으로 출하된다.**
# 2026-08-19 발연은 이 상태에서 났다.
PROTECT = {
    'Max_Temperature_Limit': 65,          # °C (기본 70 보다 낮게)
    'Protection_Current': 320,            # ≈2.1A. 12V 정격 900mA 의 2.3배·스톨 2.7A 아래
    'Over_Current_Protection_Time': 50,   # ×10ms = 0.5초 (기본 2초는 너무 길다)
    'Overload_Torque': 60,                # % (기본 80 → 더 이르게)
    'Protection_Time': 50,                # ×10ms = 0.5초 (기본 200=2초)
    'Protective_Torque': 20,              # 보호 후 유지 토크 [%] — 팔이 털썩 떨어지지 않게
}

# 그리퍼는 다르다. **물체를 잡고 계속 힘을 주는 것이 정상 동작**이라 과부하 임계를
# 다른 관절처럼 올리면 보호가 걸려 물체를 놓는다. 실제로 이 팔의 그리퍼는
# Overload_Torque 25% · Protection_Current 250 으로 낮게 세팅돼 출하됐다(2026-08-19
# 확인) — 의도된 값이므로 존중하고, **열만 막는다.**
PROTECT_GRIPPER = {
    'Max_Temperature_Limit': 65,          # 기본 70 → 낮춤
    'Protection_Time': 50,                # 2초 → 0.5초
    'Overload_Torque': 25,                # % — **그리퍼는 낮게.** 스톨 토크의 25% 만
                                          # 넘어도 보호가 걸리게 해 물체를 부수지도,
                                          # 서보가 무리하지도 않게 한다. 공장 기본
                                          # 80% 로 두면 꽉 잡는 대신 과열 위험이 크다.
                                          # 이 팔의 원래 그리퍼가 25% 였다(실측).
    'Protection_Current': 250,            # ≈1.6A — 다른 관절(320)보다 낮게
}


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
        self._calib_cache = None                  # 파일이 바뀌었을 수 있다 — 다시 읽게
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

        # ★ 서보 자체 보호를 켠다. 토크가 꺼진 지금(EEPROM 쓰기 가능) 해야 한다.
        # 소프트웨어 감시는 폴링 주기만큼 늦고, 이동 루프가 블로킹이면 아예 못 본다.
        # 펌웨어 보호는 그 사이를 메운다.
        try:
            for m in ALL:
                table = PROTECT_GRIPPER if m == 'gripper' else PROTECT
                for reg, val in table.items():
                    self.bus.write(reg, m, val, normalize=False)
            self.say(f'서보 보호 설정 완료 (과온 {PROTECT["Max_Temperature_Limit"]}°C · '
                     f'과전류 {PROTECT["Protection_Current"]} · 그리퍼는 과온만)')
        except Exception as e:
            self.say(f'⚠ 서보 보호 설정 실패: {type(e).__name__} — 과전류·과온 차단이 '
                     f'서보 기본값으로 남습니다')
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
        vel = self._profile_vel()
        self.bus.sync_write('Goal_Velocity', {m: vel for m in ALL}, normalize=False)
        self.bus.sync_write('Acceleration', {m: 15 for m in ALL}, normalize=False)
        self.bus.sync_write('Torque_Limit', {m: 600 for m in ALL}, normalize=False)

    def _profile_vel(self):
        """speed_pct → Goal_Velocity 유닛. 1%→17(≈1.5°/s) · 100%→254(≈22°/s).

        1 unit ≈ 0.087°/s (실측 2026-08-18: vel 120→10.4°/s · 200→17.0°/s).
        스톨 감지도 이 값으로 "속도 상한상 최대 얼마나 움직일 수 있었나"를 계산한다.
        """
        pct = self.snapshot()['speed_pct']
        return max(3, min(254, int(15 + pct / 100 * 239)))

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
            # ★ 토크를 켜는 것 자체가 위험 동작이다. 켜는 순간 서보는 마지막
            # Goal_Position 을 향해 움직인다 — 이전 이동의 목표가 남아 있으면
            # 그리로 튄다(2026-08-19: 그리퍼가 범위 밖이라 스스로 움직여 기구가
            # 물렸다). 켜기 전에 목표를 현재 위치로 덮고, 현재 자세가 캘리브
            # 범위를 크게 벗어났으면(바깥 여유 TORQUE_ON_TOL_DEG) 거부한다.
            #
            # 검사·기록은 전부 **raw 카운트**로 한다. 정규화 읽기는 그리퍼
            # (RANGE_0_100)에서 범위 밖을 0/100 으로 **클램프해 버려** 범위 밖을
            # 볼 수 없고, 그 클램프값을 목표로 되쓰면 켜는 순간 경계까지 스스로
            # 움직인다 — 막으려던 바로 그 동작이다 (lerobot _normalize 실측).
            try:
                raw = self.bus.sync_read('Present_Position', normalize=False)
            except Exception as e:
                self.say(f'⚠ 현재 위치를 못 읽어 토크를 켜지 않습니다: {type(e).__name__}')
                return
            if self.snapshot()['calibrated']:
                cal = self._load_calib()
                if cal is None:
                    self.say('⛔ 토크 거부 — 캘리브 파일을 읽지 못해 자세 검사를 할 수 없습니다')
                    return
                tol = int(TORQUE_ON_TOL_DEG * 4095 / 360)
                for m in ALL:
                    c = cal.get(m)
                    if not c:
                        continue
                    if not (c['range_min'] - tol <= raw[m] <= c['range_max'] + tol):
                        self.say(f'⛔ 토크 거부 — {m} 현재 raw {raw[m]} 가 캘리브 범위 '
                                 f'{c["range_min"]}~{c["range_max"]} 밖. 손으로 범위 '
                                 f'안까지 옮긴 뒤 켜세요')
                        return
            self.bus.sync_write('Goal_Position', raw, normalize=False)
            # 감시(_guard)가 붙도록 플래그를 **인가 전에** 세운다. 중간에 실패하면
            # 일부만 통전된 채 플래그가 안 서서 과열·과전류 감시와 정지 버튼이
            # 전부 비켜 간다.
            with self.lock:
                self.state['torque'] = True
            # 한 서보씩 순차 인가 — 6개 동시 돌입 전류로 전원이 주저앉아 보드가
            # USB에서 떨어진 실측(2026-08-14 15:51)이 있다
            import time as _t
            try:
                for m in ALL:
                    self.bus.enable_torque(m)
                    _t.sleep(0.15)
            except Exception as e:
                try:
                    self.bus.disable_torque()
                except Exception:
                    pass
                with self.lock:
                    self.state['torque'] = False
                self.say(f'⚠ 토크 인가 실패({type(e).__name__}) — 전체를 도로 내렸습니다')
                return
            self.say('토크 ON')
        else:
            self.bus.disable_torque()
            with self.lock:
                self.state['torque'] = False
            self.say('토크 OFF')

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
        self._calib_cache = None                  # 범위가 바뀌었다 — 게이트도 새로
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
        # ★ 조그도 캘리브 범위를 넘으면 막는다. goto·move_q 만 검사하면 한계
        # 근처에서 +5° 를 반복하는 경로가 뚫려 있다 — 범위 밖 목표는 서보가 갈 수
        # 있는 데까지 가서 나머지를 계속 미는 구조(사고와 동일)다.
        if joint in ARM:
            cur_all = self.bus.sync_read('Present_Position', ARM)
            cur = cur_all[joint]
            probe = dict(cur_all)
            probe[joint] = cur + delta
            why, _bad = self._clamp_to_calib(probe)
            if why:
                self.say(f'⛔ 조그 거부 — {why}')
                return
            tgt = cur + delta
        else:                                     # gripper — 정규화 0~100
            cur = self.bus.sync_read('Present_Position', [joint])[joint]
            tgt = max(0.0, min(100.0, cur + delta))
            if abs(tgt - cur) < 0.5:
                self.say(f'⛔ {joint} 가 이미 한계입니다 ({cur:.1f})')
                return
        self.bus.sync_write('Goal_Position', {joint: tgt})
        self.say(f'{joint} {delta:+.0f}')

    def _do_stop_test(self, joint, target, wait_s):
        """이동을 걸고 잠시 뒤 스스로 정지 — 워커 안에서 재므로 HTTP·폴링 지연이 없다.

        진단용이지만 팔을 움직이는 명령이므로 다른 이동과 **같은 게이트**를 탄다.
        종전엔 범위 검사도, 대기 상한도, 중단 확인도 없었다 — 임의 목표·임의
        대기시간을 받아 워커를 통째로 재우는 동안 온도·전류 감시와 정지 버튼이
        전부 멎는, 사고와 같은 조건을 만들 수 있었다.
        """
        if not (self.snapshot()['calibrated'] and self.snapshot()['torque']):
            self.say('⚠ 캘리브·토크 ON 후에 쓸 수 있어요')
            return
        target = float(target)
        if joint in ARM:
            try:
                cur_all = self.bus.sync_read('Present_Position', ARM)
            except Exception:
                self.say('⛔ 거부 — 현재 위치를 읽지 못해 범위 검사를 할 수 없습니다')
                return
            probe = dict(cur_all); probe[joint] = target
            why, _bad = self._clamp_to_calib(probe)
            if why:
                self.say(f'⛔ 거부 — {why}')
                return
        elif not (0.0 <= target <= 100.0):
            self.say(f'⛔ 거부 — gripper 목표 {target:.1f} 가 0~100 밖')
            return
        wait_s = min(float(wait_s), STOP_TEST_MAX_S)
        rd = lambda: self.bus.sync_read('Present_Position', [joint])[joint]
        try:
            p0 = rd()
        except Exception:
            self._kill_torque('stop_test 시작 읽기 실패 — 통신 이상')
            return
        t0 = time.monotonic()
        self.bus.sync_write('Goal_Position', {joint: target})
        # 통짜 sleep 금지 — 자는 동안 abort(정지 버튼)와 과전류를 못 본다.
        hi = 0
        end = t0 + wait_s
        while time.monotonic() < end:
            if self.abort.is_set():
                self._do_stop()
                return
            try:
                c = abs(self.bus.read('Present_Current', joint, normalize=False))
            except Exception:
                c = 0
            hi = hi + 1 if c >= CURRENT_STOP else 0
            if hi >= CURRENT_HOLD:
                self._kill_torque(f'과전류 {joint}={c} (stop_test 중, 임계 {CURRENT_STOP})')
                return
            time.sleep(0.1)
        try:
            p1 = rd(); t1 = time.monotonic()
            self._do_stop()                              # 실제 정지 경로 그대로
            t2 = time.monotonic()
            time.sleep(1.5)
            p2 = rd()
        except Exception:
            self._kill_torque('stop_test 측정 읽기 실패 — 통신 이상')
            return
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
        # 켜기 전에 목표를 현재 위치로 덮는다 — 이전 목표가 남아 있으면 토크가
        # 들어가는 순간 그리로 튄다 (_do_torque 와 같은 이유). 검사·기록은 raw 로:
        # 정규화 읽기는 범위 밖을 0/100 으로 클램프해 버려, 그 값을 목표로 되쓰면
        # 켜는 순간 경계까지 스스로 움직인다(2026-08-19 기구가 물린 사고 형태).
        raw_g = self.bus.sync_read('Present_Position', ['gripper'],
                                   normalize=False)['gripper']
        cal = self._load_calib()
        if cal is None or 'gripper' not in cal:
            self.say('⛔ 거부 — 캘리브 파일을 읽지 못해 그리퍼 자세 검사를 할 수 없습니다')
            return
        c = cal['gripper']
        tol = int(TORQUE_ON_TOL_DEG * 4095 / 360)
        if not (c['range_min'] - tol <= raw_g <= c['range_max'] + tol):
            self.say(f'⛔ 거부 — gripper 현재 raw {raw_g} 가 캘리브 범위 '
                     f'{c["range_min"]}~{c["range_max"]} 밖. 손으로 되돌린 뒤 시도하세요')
            return
        self.bus.sync_write('Goal_Position', {'gripper': raw_g}, normalize=False)
        self.bus.enable_torque('gripper')
        # ★ 그리퍼 하나만 켜도 통전은 통전이다 — 플래그를 세워야 _guard 의
        # 과열·과전류 감시가 붙는다. 파지 유지가 정확히 지속 부하 상황이라 이
        # 모터가 가장 감시가 필요한데, 종전엔 여기서 플래그가 안 서서 무기한
        # 무감시 통전이었다. 물체를 놓치면 안 되므로 토크를 도로 내리지는 않는다.
        with self.lock:
            self.state['torque'] = True
        before = self.bus.sync_read('Present_Position', ['gripper'])['gripper']
        tgt = max(0.0, min(100.0, before + delta))      # 정규화 0~100 을 넘지 않게
        self.bus.sync_write('Goal_Position', {'gripper': tgt})
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
        # ★ 슬라이더도 캘리브 범위를 넘으면 막는다. 여기는 목표만 쓰고 반환해서
        # 이동 감시가 없다 — 범위 밖으로 보내면 아무도 못 잡는 스톨이 된다.
        if joint in ARM:
            try:
                cur = self.bus.sync_read('Present_Position', ARM)
            except Exception:
                self.say('⛔ 거부 — 현재 위치를 읽지 못해 범위 검사를 할 수 없습니다')
                return
            probe = dict(cur); probe[joint] = float(value)
            why, _bad = self._clamp_to_calib(probe)
            if why:
                self.say(f'⛔ 거부 — {why}')
                return
        elif not (0.0 <= float(value) <= 100.0):   # gripper — 정규화 0~100
            self.say(f'⛔ 거부 — gripper 목표 {float(value):.1f} 가 0~100 밖')
            return
        self.bus.sync_write('Goal_Position', {joint: float(value)})
        self.say(f'{joint} → {float(value):.0f}')

    def _kill_torque(self, why):
        """스톨·과전류에서의 정지. **위치 명령을 쓰지 않고 토크를 끊는다.**

        데이터시트 7-11: 과부하·과전류 보호는 "위치 명령을 다시 보내면 플래그가
        해제된다". 그런데 보간 이동은 20ms마다 Goal_Position 을 쓴다 — 서보가 2초를
        견디고 스스로 출력을 껐는데 초당 50번 "다시 가라"고 명령해 **보호를 계속
        풀어 준다.** 2026-08-19 발연은 이 구조 때문이었다. 막힌 상황에서 _do_stop()
        처럼 현재 위치를 다시 쓰는 것조차 보호를 해제시키므로, 여기서는 토크 자체를
        내린다.
        """
        self.abort.clear()
        try:
            self.bus.disable_torque()
        except Exception:
            pass
        with self.lock:
            self.state['torque'] = False
        self.say(f'⛔ {why} — 토크를 내렸습니다 (위치 명령을 보내지 않습니다)')

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

    def _clamp_to_calib(self, target, margin=LIMIT_MARGIN_DEG):
        """목표가 **캘리브 범위** 안인지 보고, 벗어나면 (거부 사유, 관절)을 돌려준다.

        margin 은 범위 안쪽으로 남기는 여유[°]다. 음수를 주면 바깥 허용이 된다 —
        토크 켜기 검사처럼 "크게 벗어났는지"만 볼 때 쓴다.

        IK 는 URDF 관절 한계로 해를 내는데 그 값이 서보 실측 범위보다 넓다
        (2026-08-19 실측: shoulder_pan ±98.9° 인데 URDF 는 ±110°). 범위 밖을
        명령하면 lerobot 은 클램프 없이 그대로 내보내고(_unnormalize 의 DEGREES
        분기), 이후는 펌웨어 한계에 부딪혀 스톨 감지가 전 관절 토크를 떨어뜨리는
        것으로 끝난다 — 애초에 안 보내는 편이 낫다.

        경계는 arm_lib.calib_bounds — lerobot DEGREES 정규화(범위 중점 기준,
        360/4095)와 같은 식이다. 캘리브 파일을 못 읽으면 **거부**한다(fail-closed).
        여기가 뚫리면 이 게이트에 기대는 jog·goto·move_q 전부가 조용히 무방비가 된다.
        """
        cal = self._load_calib()
        if cal is None:
            return '캘리브 파일을 읽지 못해 범위 검사를 할 수 없습니다', None
        bounds = arm_lib.calib_bounds(cal)
        for j in ARM:
            if j not in bounds:
                continue
            lo = bounds[j][0] + margin
            hi = bounds[j][1] - margin
            v = target[j]
            if v < lo:
                return f'{j} 목표 {v:.1f}° 가 캘리브 하한 {lo:.1f}° 밖', j
            if v > hi:
                return f'{j} 목표 {v:.1f}° 가 캘리브 상한 {hi:.1f}° 밖', j
        return None, None

    def _load_calib(self):
        """캘리브 파일을 읽어 캐시한다 — 조그 버튼마다 디스크를 파싱하지 않게.

        연결·저장이 캐시를 비운다. 못 읽으면 None — 호출부는 거부해야 한다.
        """
        cal = getattr(self, '_calib_cache', None)
        if cal is None:
            try:
                cal = json.loads(self.calib_path().read_text())
            except Exception:
                return None
            self._calib_cache = cal
        return cal

    def _interp(self, cur, target, seconds):
        """cur → target 으로 보간 이동. 도달하면 True, 중단하면 False."""
        steps = max(2, int(seconds * 50))
        watch = None                              # 스톨 감지용 직전 위치
        self._hi = 0                              # 과전류 연속 관측 횟수
        self._peak = {}                           # 이번 이동의 관절별 전류 피크
        for i in range(1, steps + 1):
            if self.abort.is_set():               # 정지 버튼 — 즉시 끊는다
                self._do_stop()
                return False

            # ★ 이동 **중** 스톨 감지. 보간이 끝난 뒤에만 확인하면 그때까지는 막힌
            # 채로 계속 민다 — 서보가 타는 것은 그 구간이다(2026-08-19 사고).
            # 0.5초마다 보고, 위치가 안 변하는데 목표가 남아 있으면 즉시 끊는다.
            if i % 10 == 0:
                # 전류는 위치보다 먼저 반응한다. 막히면 즉시 튀므로 더 자주 본다.
                try:
                    cur_a = self.bus.sync_read('Present_Current', ARM, normalize=False)
                except Exception:
                    cur_a = None
                if cur_a:
                    # 이동 중 전류 피크를 기록한다. 임계(CURRENT_STOP)는 데이터시트
                    # 추정이라 **정상 동작 값을 알아야** 맞출 수 있는데, _guard 는
                    # 6초 주기라 짧은 이동에서는 피크를 통째로 놓친다.
                    for j, v in cur_a.items():
                        a = abs(v)
                        if a > self._peak.get(j, 0):
                            self._peak[j] = a
                    over = {j: abs(v) for j, v in cur_a.items() if abs(v) >= CURRENT_STOP}
                    self._hi = self._hi + 1 if over else 0
                    if self._hi >= CURRENT_HOLD:
                        self._kill_torque(f'과전류 {over} (임계 {CURRENT_STOP}≈'
                                          f'{CURRENT_STOP*6.5/1000:.1f}A)')
                        return False

            if i % 25 == 0:
                try:
                    now = self.bus.sync_read('Present_Position', ARM)
                except Exception:
                    # 읽기가 안 되면 통신이 흔들리는 것이다 — 그동안에도 서보는
                    # 마지막 목표를 향해 계속 민다. _do_stop 은 위치를 다시 써서
                    # 서보 보호를 해제시키고, 그 안의 읽기도 같이 실패하면 토크가
                    # 켜진 채 옛 목표만 남는다. 여기서는 무조건 토크를 끊는다.
                    self._kill_torque('이동 중 읽기 실패 — 통신 이상, 토크를 끊습니다')
                    return False
                if watch is not None:
                    # ★ 관절별로 본다. 전체 max 로 묶으면(종전 방식) 한 관절이
                    # 막혀도 다른 관절이 움직이는 동안은 "moved 가 크다"로 읽혀
                    # 감지가 안 된다 — 사고가 정확히 그 모양이었다(wrist_roll 만
                    # 막히고 나머지는 내려가는 중).
                    #
                    # 비교 기준은 최종 목표가 아니라 **이 순간의 보간 목표**다.
                    # 최종 목표와 비교하면 이동 초반엔 누구나 멀리 있어 오탐하고,
                    # 임계를 키우면 그만큼 늦게 잡는다. 보간 목표는 막힌 관절에서만
                    # 앞서 나가므로 초반에도 정확하다.
                    ai = i / steps
                    si = ai * ai * (3 - 2 * ai)
                    goal_now = {j: cur[j] + (target[j] - cur[j]) * si for j in ARM}
                    lag = {j: abs((now[j] - goal_now[j] + 180) % 360 - 180)
                           for j in ARM}
                    # 두 갈래로 잡는다:
                    # · 완전 스톨 — 거의 안 움직였는데 보간 목표가 앞서 있다
                    # · 기는 스톨 — 움직이고는 있어도 뒤처짐이 STALL_LAG_DEG 를
                    #   넘었다. 마찰로 1°/s 씩 기는 부분 스톨은 이동량 조건을
                    #   통과해 버리는데, 그 상태도 스톨 전류를 흘려 서보를 태운다.
                    #   단 **속도 상한 때문에** 뒤처지는 정상 이동은 면제한다 —
                    #   상한이 허용하는 이동량의 절반 이상을 소화하고 있으면
                    #   막힌 게 아니라 최고 속도로 가는 중이다. (진짜 부분 스톨은
                    #   전류 감시(0.2초)가 더 먼저 잡는 것이 보통이다.)
                    win_cap = self._profile_vel() * 0.087 * 0.5   # 0.5초 창 최대 이동 [°]
                    stuck = [j for j in ARM
                             if (abs(now[j] - watch[j]) < STALL_MOVE_DEG
                                 and lag[j] > STALL_GAP_DEG)
                             or (lag[j] > STALL_LAG_DEG
                                 and abs(now[j] - watch[j]) < 0.5 * win_cap)]
                    if stuck:
                        worst = max(stuck, key=lambda j: lag[j])
                        self._kill_torque(f'스톨 — {worst} 가 보간 목표에서 '
                                          f'{lag[worst]:.1f}° 뒤처져 있습니다. '
                                          f'간섭을 확인하세요')
                        return False
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
            # 통신 이상 — _do_stop 은 위치 재전송으로 서보 보호를 해제시킨다. 끊는다.
            self._kill_torque('도달 확인 읽기 실패 — 통신 이상, 토크를 끊습니다')
            return False
        gap = max(abs((fin[j] - target[j] + 180) % 360 - 180) for j in ARM)
        if gap > STALL_GAP_DEG:
            worst = max(ARM, key=lambda j: abs((fin[j] - target[j] + 180) % 360 - 180))
            self._kill_torque(f'목표에서 {gap:.1f}° 남음 ({worst}) — 간섭을 확인하세요')
            return False
        return True

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

        # ★ 캘리브 범위를 벗어나는 목표는 아예 보내지 않는다.
        why, _bad = self._clamp_to_calib(target)
        if why:
            self.say(f'⛔ 이동 거부 — {why}. IK 는 URDF 한계로 해를 내는데 서보 실측'
                     f' 범위가 더 좁습니다')
            return

        # ★ 회전을 먼저, 이동을 나중에.
        #
        # 2026-08-19 사고의 순서 문제다. wrist_roll 이 98°(누운 자세)라 죠가 좌우로
        # 벌어진 채 팔이 내려가 **한쪽 턱이 책상에 닿았고**, 그 상태에서 IK 가 roll 을
        # 180° 로 돌리라고 명령했다. 눌린 죠는 돌 수 없다 — 82° 를 남기고 스톨,
        # 과열, 발연으로 이어졌다.
        #
        # 5관절을 한꺼번에 명령하면 "눌린 채 돌리기"가 언제든 다시 나온다. roll 변화가
        # 크면 나머지를 그대로 둔 채 **roll 만 먼저** 돌리고, 그다음 본 이동을 한다.
        # 각 단계마다 스톨·과전류 감시가 그대로 걸린다.
        roll_gap = abs((target['wrist_roll'] - cur['wrist_roll'] + 180) % 360 - 180)
        if roll_gap > ROLL_FIRST_DEG:
            self.say(f'wrist_roll 을 {roll_gap:.0f}° 먼저 돌립니다 '
                     f'(죠가 눌린 채 회전하지 않도록)')
            first = dict(cur)
            first['wrist_roll'] = target['wrist_roll']
            if not self._interp(cur, first, max(1.5, roll_gap / 60)):
                return                            # 회전이 막혔으면 본 이동을 하지 않는다
            try:
                cur = self.bus.sync_read('Present_Position', ARM)
            except Exception:
                self._kill_torque('회전 후 읽기 실패 — 통신 이상, 토크를 끊습니다')
                return

        if self._interp(cur, target, seconds):
            pk = getattr(self, '_peak', {})
            top = sorted(pk.items(), key=lambda x: -x[1])[:3]
            note = ' · '.join(f'{j[:8]}={v}' for j, v in top if v) or '전류 0'
            with self.lock:
                self.state['last_peak'] = dict(pk)
            self.say(f'이동 완료 — 전류피크 {note} (임계 {CURRENT_STOP})')

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
        curs = {}
        for m in ALL:
            try:
                curs[m] = abs(self.bus.read('Present_Current', m, normalize=False))
            except Exception:
                pass
        if not temps:
            return
        with self.lock:
            self.state['temp'] = temps
            if curs:
                self.state['current'] = curs
        hot_i = {m: c for m, c in curs.items() if c >= CURRENT_STOP}
        if hot_i:
            try:
                self.bus.disable_torque()
            except Exception:
                pass
            with self.lock:
                self.state['torque'] = False
            self.say(f'⛔ 과전류 자동 정지 — {hot_i} (임계 {CURRENT_STOP})')
            return
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
