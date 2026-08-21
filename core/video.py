"""JPEG 낱장 → H.264 MP4 (data URI). **화질을 올리면서 용량은 줄이는 자리.**

낱장 JPEG을 시각마다 갈아 끼우는 방식은 되감기에는 편하지만 **같은 그림을 매번 통째로
싣는다.** 로봇 기록은 연속 프레임이 거의 같으므로 비디오 코덱이 훨씬 유리하다.

실측(640×480 · 손목캠 · 252초):

| | JPEG 낱장 | H.264 |
|---|---|---|
| 2fps 304장 | 4.30MB | **1.72MB** |
| 4fps 353장 | 4.94MB | **1.91MB** |

그래서 같은 용량에 **픽셀을 8.7배**(217×163 → 640×480) 키울 수 있다. 옛 화면이 217×163이던
것은 녹화기가 찍는 순간 0.34배로 줄여 저장했기 때문이고, 그건 나중에 되살릴 수 없다.

## 되감기가 되어야 한다

이 화면은 시간축을 마음대로 긁는다. 그래서 **키프레임을 촘촘히 박는다**(`g=fps`, 1초마다).
안 그러면 `currentTime`을 옮길 때마다 디코더가 앞 키프레임부터 다시 풀어 화면이 늦는다.
위 표의 크기는 그 촘촘한 키프레임까지 포함한 값이다 — 실제로 써도 저 크기다.

코덱은 **H.264**다. AV1이 더 작지만 브라우저 지원이 고르지 않고, 이 페이지는 남의 기계에서
열린다. 되는 것이 작은 것보다 낫다.
"""
import base64
import io
import pathlib
import shutil

from PIL import Image


def even(n):
    """인코더는 홀수 변을 싫어한다."""
    return n - (n % 2)


def encode(frames, fps, size=None, crf=26, preset='slow'):
    """[{'t','b'(base64 JPEG)}] → (data URI, 바이트 수, (w,h))

    `frames`는 시각순이어야 한다. 반환되는 URI는 `<video src=...>`에 그대로 넣는다.
    """
    import av                                    # 무거워서 쓸 때만 부른다
    if not frames:
        return None, 0, None
    first = Image.open(io.BytesIO(base64.b64decode(frames[0]['b'])))
    w, h = size or first.size
    size = (even(w), even(h))

    buf = io.BytesIO()
    container = av.open(buf, 'w', format='mp4')
    st = container.add_stream('libx264', rate=fps)
    st.width, st.height, st.pix_fmt = size[0], size[1], 'yuv420p'
    st.options = {
        'crf': str(crf),
        'preset': preset,
        # 키프레임 간격 = 1초. 되감기 반응이 여기서 갈린다.
        'g': str(max(1, int(fps))),
        # moov를 앞으로 — data URI로 통째로 들고 있으므로 크게 중요하진 않지만,
        # 브라우저가 첫 프레임을 그리기 전에 파일 끝까지 훑지 않게 한다.
        'movflags': 'faststart',
    }
    for f in frames:
        img = Image.open(io.BytesIO(base64.b64decode(f['b']))).convert('RGB')
        if img.size != size:
            img = img.resize(size)
        container.mux(st.encode(av.VideoFrame.from_image(img)))
    container.mux(st.encode())
    container.close()

    data = buf.getvalue()
    return 'data:video/mp4;base64,' + base64.b64encode(data).decode(), len(data), size


def resample(frames, fps, span):
    """고정 주기 격자에 맞춰 고른다. 비디오는 가변 주기를 못 담는다.

    각 격자 시각 이하에서 **가장 늦은** 프레임을 쓴다(영차 유지). 없는 시각을 만들어
    내지 않고, 프레임이 겹칠 뿐이다 — 겹치는 것은 코덱이 거의 공짜로 삼킨다.
    """
    if not frames:
        return []
    out, i, n = [], 0, len(frames)
    for k in range(int(span * fps) + 1):
        t = k / fps
        while i + 1 < n and frames[i + 1]['t'] <= t + 1e-6:
            i += 1
        out.append(frames[i])
    return out


def build(by_run, fps, budget_mb, size=None, crf=26, crf_by_cam=None,
          out_dir=None, url_base='media'):
    """시행별·카메라별 프레임 → 비디오. 예산을 넘으면 **화질부터** 낮춘다.

    프레임을 솎으면 재생이 끊기고, 해상도를 낮추면 화질이 통째로 내려간다. 그런데
    H.264의 `crf`는 **그 사이**를 준다 — 같은 해상도·같은 프레임 수로 용량만 줄인다.
    그래서 예산 초과 시 `crf`를 먼저 올리고, 그래도 안 되면 그때 해상도를 내린다.

    `crf_by_cam`으로 **채널마다 화질을 다르게** 준다. 예산은 하나인데 채널이 여럿이면
    똑같이 깎을 이유가 없다 — 확대해서 들여다볼 채널(손목캠·정면)과 배경으로만 두는
    채널(화면 녹화·시뮬 미러)은 필요한 화질이 다르다. 값은 기준 crf에 **더하는 차**다
    (실측 2026-08-21: 같은 원본이 crf 28에 5.9MB, crf 40에 2.1MB — 3배 차이).

    `out_dir`를 주면 data URI 대신 **옆 파일로 뺀다.** 그러면 세 가지가 한꺼번에 풀린다 —
    base64가 33% 부풀리던 것이 없어지고, 한 장에 다 넣어야 하는 예산 제약이 사라지며,
    브라우저가 보는 채널만 받아 온다(첫 화면이 빨라진다). 대신 단일 파일이 아니게 되므로
    아티팩트로 발행할 화면은 `out_dir` 없이 그대로 묻어 넣는다.
    """
    bump = crf_by_cam or {}
    if out_dir is not None:
        return _build_files(by_run, fps, size, crf, bump, pathlib.Path(out_dir), url_base)
    budget = budget_mb * 1024 * 1024
    for attempt, (c, sc) in enumerate([(crf, 1.0), (crf + 6, 1.0), (crf + 6, 0.75),
                                       (crf + 10, 0.75), (crf + 10, 0.5)]):
        out, total = {}, 0
        for rid, cams in by_run.items():
            out[rid] = {}
            for cam, frames in cams.items():
                if not frames:
                    continue
                sz = None
                if sc != 1.0:
                    w, h = Image.open(io.BytesIO(base64.b64decode(frames[0]['b']))).size
                    sz = (int(w * sc), int(h * sc))
                uri, nbytes, dim = encode(frames, fps, sz, crf=c + bump.get(cam, 0))
                if uri:
                    out[rid][cam] = {'src': uri, 'fps': fps, 'w': dim[0], 'h': dim[1],
                                     't0': frames[0]['t'], 'n': len(frames)}
                    total += nbytes
        if total <= budget:
            if attempt:
                print(f'  예산 맞추려 화질을 낮췄다 — crf {c} · 해상도 {sc:.0%}')
            return out, total
        print(f'  {total/1e6:.1f}MB > {budget_mb}MB — crf {c} · 해상도 {sc:.0%}로 다시')
    print('  ! 예산을 못 맞췄다 — 가장 낮은 설정으로 내보낸다')
    return out, total


def _build_files(by_run, fps, size, crf, bump, out_dir, url_base):
    """영상을 옆 파일로 뺀다. 예산 조이기가 없으므로 **화질을 낮출 이유도 없다.**

    디렉터리는 매번 비우고 다시 쓴다 — 시행이 빠졌는데 파일만 남으면 어느 것이 지금
    화면의 것인지 알 수 없게 된다.
    """
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)
    out, total = {}, 0
    for rid, cams in by_run.items():
        out[rid] = {}
        for cam, frames in cams.items():
            if not frames:
                continue
            uri, nbytes, dim = encode(frames, fps, size, crf=crf + bump.get(cam, 0))
            if not uri:
                continue
            name = f'{rid.replace("/", "_")}_{cam}.mp4'
            (out_dir / name).write_bytes(base64.b64decode(uri.split(',', 1)[1]))
            out[rid][cam] = {'src': f'{url_base}/{name}', 'fps': fps,
                             'w': dim[0], 'h': dim[1], 't0': frames[0]['t'],
                             'n': len(frames), 'bytes': nbytes}
            total += nbytes
    return out, total
