"""MCAP을 읽는 **중립 층.** 로봇도 과제도 모른다.

여기 있는 것은 어떤 ROS 2 기록에도 쓰이는 것들뿐이다 — 파일 열기, 시각으로 값 찾기,
TF 체인 합성, 카메라 프레임 굽기, 용량 예산 맞추기. **관절 이름·판정 규칙·토픽 의미는
여기 없다.** 그런 것은 `projects/<이름>/mcap_read.py`가 가진다.

처음엔 이 파일과 프로젝트 코드가 한 덩어리(`core/mcap_read.py`)였다. SLAM을 태우려다
드러났다 — `steps_of`가 `/joint_states`와 `P.ARM_ORDER`를, 판정이 `/gt/trash`를 전제해서
SLAM 기록으로는 import조차 못 했다. "core는 로봇도 과제도 모른다"고 적어 뒀던 것이
사실이 아니었다. 이 파일이 그 약속을 실제로 지키는 자리다.

## `/tf`를 객체로 쌓지 말 것

`mcap_ros2`가 돌려주는 메시지는 필드마다 파이썬 인스턴스가 달린 중첩 구조라 원본의
100배로 부푼다. `/tf`는 `robot_state_publisher`가 관절 상태마다 전 링크를 재발행해서
1000Hz를 넘기도 한다(SLAM 기록 실측 82,042건·1045Hz). 그래서 여기서 **읽는 순간
9-튜플로 접는다.**

이걸 안 지켜서 사고가 났다(2026-08-13). 상한 없이 띄운 `slam read`가 392MB를 읽다
RAM 25.6GB까지 자랐고, `systemd-oomd`가 진범 대신 gnome-shell을 죽여 데스크톱이
무너졌다. **표본을 솎아서는 못 고친다** — `/tf`를 시간 기준으로 솎으면 판정이 깨진다
(실측 8mm → 240mm). 줄일 것은 개수가 아니라 한 건의 크기다.
실측: 객체 보관 2.65GB → 튜플 보관 0.13GB, 29.8초 → 9.5초. 판정값은 동일하다.
"""
import base64
import bisect
import io
import math

from mcap.reader import make_reader
from mcap_ros2.reader import DecoderFactory, read_ros2_messages

# `/tf` 한 건을 접은 모양. `tf_pose`가 이 순서로 푼다.
TF_FIELDS = ('child', 'parent', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))


def xyz(p):
    return [round(p.x, 4), round(p.y, 4), round(p.z, 4)]


def at(series, t, max_age, eps=1e-6):
    """t 시점의 값. 같은 시각에 실린 것까지 포함해 가장 최근 것을 고른다.

    `max_age`보다 오래된 표본은 버린다. 끌고 오면 화면이 "그때 거기 있었다"고 말하게
    되는데, 그건 기록이 아니라 추정이다.
    """
    i = bisect.bisect_right(series, t + eps, key=lambda p: p[0])
    if not i:
        return None
    ts, v = series[i - 1]
    return v if t - ts <= max_age else None


def _tf_light(msg):
    """TFMessage → 9-튜플 목록. 객체를 쥐고 있지 않는 것이 요점이다."""
    return [(tr.child_frame_id, tr.header.frame_id,
             tr.transform.translation.x, tr.transform.translation.y,
             tr.transform.translation.z,
             tr.transform.rotation.x, tr.transform.rotation.y,
             tr.transform.rotation.z, tr.transform.rotation.w)
            for tr in msg.transforms]


def to_jpeg(m, topic, scale, quality):
    """카메라 메시지 → JPEG 바이트. 압축 토픽이면 그대로, 원본이면 여기서 줄여 굽는다."""
    if topic.endswith('/compressed'):
        return bytes(m.data)
    from PIL import Image                      # 원본 이미지가 실린 bag에서만 필요하다
    import numpy as np
    ch = {'rgb8': 3, 'bgr8': 3, 'mono8': 1, 'rgba8': 4, 'bgra8': 4}.get(m.encoding)
    if ch is None:
        return None
    a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, ch)
    if m.encoding.startswith('bgr'):
        a = a[:, :, ::-1] if ch == 3 else a[:, :, [2, 1, 0, 3]]
    img = Image.fromarray(a.squeeze())
    img = img.resize((max(1, int(m.width * scale)), max(1, int(m.height * scale))))
    buf = io.BytesIO()
    img.convert('RGB').save(buf, 'JPEG', quality=quality)
    return buf.getvalue()


def load(path, cams, jpeg, keep=None):
    """MCAP 한 개 → (메타데이터, 토픽별 시계열, 카메라 프레임).

    `cams`  : 토픽 → 화면에서 부를 이름
    `jpeg`  : {'scale', 'quality'} — 원본 이미지를 구울 때만 쓴다
    `keep`  : `(topic, msg) → 보관할 값`. 프로젝트가 더 접고 싶을 때 넘긴다.
              `/tf`·`/tf_static`은 이 훅과 무관하게 **항상** 튜플로 접는다.
    """
    series, frames, meta = {}, {}, {}
    with path.open('rb') as f:
        # 메타데이터와 메시지를 한 번에 훑으려면 리더를 직접 세워야 한다. 이때
        # ROS 2 해독기를 함께 물려야 스키마가 파일 안에 있어도 메시지가 풀린다.
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for md in reader.iter_metadata():
            if md.name == 'run':
                meta = dict(md.metadata)
        for m in read_ros2_messages(reader):
            topic, t = m.channel.topic, m.log_time_ns / 1e9
            if topic in cams:
                b = to_jpeg(m.ros_msg, topic, jpeg['scale'], jpeg['quality'])
                if b:
                    frames.setdefault(cams[topic], []).append(
                        {'t': round(t, 2), 'b': base64.b64encode(b).decode()})
            elif topic in ('/tf', '/tf_static'):
                series.setdefault(topic, []).append((t, _tf_light(m.ros_msg)))
            else:
                series.setdefault(topic, []).append(
                    (t, keep(topic, m.ros_msg) if keep else m.ros_msg))
    for name in set(cams.values()):
        frames.setdefault(name, [])
    return meta, series, frames


def tf_tree(series):
    """`/tf`·`/tf_static`을 자식 프레임으로 색인한다. 고정 관절은 static 쪽에만 있다."""
    dyn, static = {}, {}
    for t, rows in series.get('/tf', []):
        for tr in rows:
            dyn.setdefault(tr[0], []).append((t, tr))
    for _, rows in series.get('/tf_static', []):
        for tr in rows:
            static[tr[0]] = tr
    return dyn, static


def qrot(qx, qy, qz, qw, v):
    """사원수로 벡터 v를 돌린다 (v + 2·qv × (qv × v + w·v))."""
    u = (qx, qy, qz)
    c1 = (u[1] * v[2] - u[2] * v[1] + qw * v[0],
          u[2] * v[0] - u[0] * v[2] + qw * v[1],
          u[0] * v[1] - u[1] * v[0] + qw * v[2])
    c2 = (u[1] * c1[2] - u[2] * c1[1],
          u[2] * c1[0] - u[0] * c1[2],
          u[0] * c1[1] - u[1] * c1[0])
    return (v[0] + 2 * c2[0], v[1] + 2 * c2[1], v[2] + 2 * c2[2])


def tf_pos(tree, target, root, t, max_age):
    """t 시점에서 `root` 기준 `target` 원점의 위치.

    `/tf`는 **부모→자식 한 칸씩만** 싣는다 — 손끝까지의 변환이 통째로 실려 오는 일은
    없다. 그래서 자식에서 부모로 거슬러 올라가며 합성한다(`tf2`의 `lookup_transform`이
    하는 일). 변환기가 만든 파일은 한 칸이면 끝나고, `ros2 bag record`가 받아 적은
    파일은 관절 수만큼 거슬러 올라간다. tf2와 0.00000mm 일치를 확인한 코드다.
    """
    dyn, static = tree
    p, frame = (0.0, 0.0, 0.0), target
    for _ in range(64):                    # 트리가 고리를 이루면 여기서 멈춘다
        if frame == root:
            return [round(v, 4) for v in p]
        tr = at(dyn.get(frame, []), t, max_age) or static.get(frame)
        if tr is None:
            return None
        p = qrot(tr[5], tr[6], tr[7], tr[8], p)
        p = (p[0] + tr[2], p[1] + tr[3], p[2] + tr[4])
        frame = tr[1]
    return None


def rebase(rec, frames):
    """시행의 0초를 **첫 단계**로 옮긴다. 카메라와 관절을 한 시계 위에 둔 채로.

    bag의 0초는 녹화가 깨어난 순간이라 무대를 세우는 시간이 앞에 붙어 있다. 화면의
    0초는 시행이 시작한 순간이어야 하므로 여기서 한 번 민다. **두 계열을 같은 값만큼**
    미는 것이 요점이다 — 각자 0으로 맞추면 영상과 관절이 어긋난 채 겹쳐 재생된다
    (캡스톤에서 실제로 4.5초 어긋나 있었다).
    """
    t0 = rec['steps'][0]['t']
    for s in rec['steps']:
        s['t'] = round(s['t'] - t0, 1)
    rec['dur'] = rec['steps'][-1]['t']
    for cam, fs in frames.items():
        for f in fs:
            f['t'] = round(f['t'] - t0, 2)
        # 시행이 시작하기 전 장면은 화면의 시간축 밖이라 어차피 안 보인다
        frames[cam] = [f for f in fs if f['t'] >= 0]


def thin(seq, keep):
    """시각이 고르게 남도록 솎는다. 앞뒤를 자르면 시작·끝 장면이 통째로 사라진다."""
    if keep >= len(seq):
        return seq
    step = len(seq) / keep
    return [seq[min(len(seq) - 1, int(i * step))] for i in range(keep)]


def fit_budget(by_run, budget_mb, weights=None):
    """아티팩트 한 장에 다 들어가야 한다. 넘으면 솎되, **솎은 사실을 반드시 찍는다.**

    `weights`로 카메라마다 남길 비중을 다르게 준다 — 캡스톤은 파지·운반이 손목캠에서만
    보여서 전방캠을 더 깎는다. 같은 비율로 깎으면 정작 봐야 할 쪽이 끊긴다.
    """
    total = sum(len(f['b']) for r in by_run.values() for cam in r for f in r[cam])
    budget = budget_mb * 1024 * 1024
    if total <= budget:
        return
    w = weights or {}
    wt = lambda cam: w.get(cam, 1.0)                                    # noqa: E731
    weighted = sum(len(f['b']) * wt(cam) for r in by_run.values() for cam in r for f in r[cam])
    k = budget / weighted
    share = ' · '.join(f'{c} {min(1, k * wt(c)) * 100:.0f}%'
                       for c in sorted({c for r in by_run.values() for c in r}))
    print(f'  예산 초과 {total/1e6:.1f}MB > {budget_mb}MB — 솎아낸다 ({share})')
    for rid, r in by_run.items():
        for cam in r:
            before = len(r[cam])
            r[cam] = thin(r[cam], max(2, int(before * min(1.0, k * wt(cam)))))
            if before:
                print(f'    {rid} {cam}: {before} → {len(r[cam])}장')
