"""URDF → urdf_chain.json — 관절 기하와 손가락 패드를 함께 뽑는다.

패드까지 뽑는 이유: 그리퍼가 얼마나 벌어졌는지를 "물림각 0.41rad"이 아니라
**밀리미터 간격**으로 보여주려면 두 손가락 끝의 실제 좌표가 필요하다. 그 좌표는
URDF 콜리전 박스(`fixed_finger_tip` / `moving_finger_tip`)에 이미 실측으로 들어 있다.
각도를 임의 배율로 mm에 대응시키면 URDF와 어긋난 숫자가 화면에 뜬다.

사용: python3 extract_chain.py
"""
import json
import pathlib
import xml.etree.ElementTree as ET

URDF = pathlib.Path.home() / 'jdamr_cube_ws/src/jdamr_cube_ros/jdamr_cube_description/urdf/jdamr_cube.urdf'
OUT = pathlib.Path(__file__).parent / 'urdf_chain.json'


def triple(node, attr, default):
    if node is None or node.get(attr) is None:
        return list(default)
    return [float(v) for v in node.get(attr).split()]


def main():
    root = ET.parse(URDF).getroot()

    joints = []
    for j in root.findall('joint'):
        o = j.find('origin')
        lim = j.find('limit')
        joints.append({
            'name': j.get('name'),
            'type': j.get('type'),
            'parent': j.find('parent').get('link'),
            'child': j.find('child').get('link'),
            'xyz': triple(o, 'xyz', (0, 0, 0)),
            'rpy': triple(o, 'rpy', (0, 0, 0)),
            'axis': triple(j.find('axis'), 'xyz', (0, 0, 1)),
            'lo': float(lim.get('lower')) if lim is not None and lim.get('lower') else None,
            'hi': float(lim.get('upper')) if lim is not None and lim.get('upper') else None,
        })

    # 손가락 패드 — 이름 붙은 콜리전만. 링크 로컬 좌표라 FK 뒤에 그대로 곱하면 된다.
    pads = []
    for link in root.findall('link'):
        for col in link.findall('collision'):
            nm = col.get('name') or ''
            if 'finger' not in nm:
                continue
            box = col.find('geometry/box')
            if box is None:
                continue
            pads.append({
                'name': nm,
                'type': 'pad',
                'parent': link.get('name'),
                'xyz': triple(col.find('origin'), 'xyz', (0, 0, 0)),
                'rpy': triple(col.find('origin'), 'rpy', (0, 0, 0)),
                'size': triple(box, 'size', (0, 0, 0)),
            })

    out = joints + pads
    OUT.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f'{OUT.name}  관절 {len(joints)} · 패드 {len(pads)}  {OUT.stat().st_size}B')
    for p in pads:
        print(f'  {p["parent"]:24s} {p["name"]:20s} xyz {p["xyz"]}')


if __name__ == '__main__':
    main()
