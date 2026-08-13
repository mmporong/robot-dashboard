"""ROS 2 `.msg` 정의를 MCAP 스키마 본문으로 조립한다.

MCAP은 스키마를 **파일 안에 넣고 다닌다** — 그래서 이 파일을 읽는 쪽은 ROS가 깔려
있지 않아도 메시지를 풀 수 있다. 대신 쓰는 쪽이 정의 본문을 채워 넣어야 한다.
`rosbag2`가 쓰는 형식과 같게 맞춘다: 최상위 정의 뒤에 의존 타입을 구분선으로 잇는다.

    <최상위 .msg 본문>
    ================================================================================
    MSG: geometry_msgs/Pose
    <그 타입의 본문>
    ...

여기서만 ROS 설치가 필요하다(`/opt/ros/*/share`에서 `.msg` 원문을 읽는다).
읽는 쪽(`read_mcap.py`)은 ROS 없이 돈다 — 그게 표준 포맷을 쓰는 이유다.
"""
import os
import pathlib
import re

SEP = '=' * 80
# 'float64[36] covariance', 'string[<=10] x' 처럼 배열 표기가 붙으므로 대괄호를 떼고 본다
_ARRAY = re.compile(r'\[[^\]]*\]$')
_PRIMITIVE = {
    'bool', 'byte', 'char', 'float32', 'float64', 'string', 'wstring',
    'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64',
}


def _share_dirs():
    """`.msg`가 있는 share 디렉터리들. ROS를 source 하지 않아도 찾도록 한다."""
    dirs = []
    for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
        if p:
            dirs.append(pathlib.Path(p) / 'share')
    dirs += sorted(pathlib.Path('/opt/ros').glob('*/share'), reverse=True)
    return [d for d in dirs if d.is_dir()]


def _find(pkg, name):
    for share in _share_dirs():
        f = share / pkg / 'msg' / f'{name}.msg'
        if f.is_file():
            return f.read_text(encoding='utf-8')
    raise FileNotFoundError(f'{pkg}/msg/{name}.msg 를 찾지 못했다 — ROS 2가 설치돼 있나')


def _deps(text, pkg):
    """본문에서 참조하는 비원시 타입을 `pkg/Name`으로 뽑는다."""
    out = []
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        # 상수 선언(`uint8 OK=0`)도 타입이 앞에 오지만 원시형이라 어차피 걸러진다
        typ = _ARRAY.sub('', parts[0])
        if typ in _PRIMITIVE:
            continue
        if '/' in typ:
            owner, name = typ.split('/')[0], typ.split('/')[-1]
        elif typ == 'Header':
            # ROS 2가 유일하게 허용하는 비수식 참조
            owner, name = 'std_msgs', 'Header'
        else:
            owner, name = pkg, typ
        out.append(f'{owner}/{name}')
    return out


def msgdef(datatype):
    """'sensor_msgs/msg/JointState' → MCAP 스키마 본문 (의존 타입까지 이어 붙인 것)."""
    pkg, name = datatype.split('/')[0], datatype.split('/')[-1]
    top = _find(pkg, name)
    chunks, seen, queue = [], set(), _deps(top, pkg)
    while queue:
        ref = queue.pop(0)
        if ref in seen:
            continue
        seen.add(ref)
        rpkg, rname = ref.split('/')
        body = _find(rpkg, rname)
        chunks.append(f'{SEP}\nMSG: {ref}\n{body}')
        queue += _deps(body, rpkg)
    return top + ''.join(chunks)
