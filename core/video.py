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


def build(by_run, fps, budget_mb, size=None, crf=26):
    """시행별·카메라별 프레임 → 비디오. 예산을 넘으면 **화질부터** 낮춘다.

    프레임을 솎으면 재생이 끊기고, 해상도를 낮추면 화질이 통째로 내려간다. 그런데
    H.264의 `crf`는 **그 사이**를 준다 — 같은 해상도·같은 프레임 수로 용량만 줄인다.
    그래서 예산 초과 시 `crf`를 먼저 올리고, 그래도 안 되면 그때 해상도를 내린다.
    """
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
                uri, nbytes, dim = encode(frames, fps, sz, crf=c)
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
