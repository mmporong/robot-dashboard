#!/usr/bin/env python3
"""Orbbec Astra S 깊이 스트림 파이썬 바인딩 — ctypes 로 C API 를 직접 부른다.

## 왜 ctypes 인가

Astra S 는 구형이라 신형 `pyorbbecsdk`(OrbbecSDK v2) 대상이 아니고, Legacy Astra SDK
2.1.3 에는 파이썬 바인딩이 없다. C API 는 함수 여섯 개면 깊이 프레임을 얻을 수 있어
별도 빌드 없이 ctypes 로 감싸는 편이 가볍다. (2026-08-18 확인: 18.04 용 바이너리가
Ubuntu 24.04 에서 의존성 결측 없이 그대로 동작한다 — SFML 뷰어 샘플만 못 쓴다.)

## 좌표 규약

`points()` 는 카메라 광학 프레임 기준 (X 오른쪽, Y 아래, Z 전방, 단위 m)으로 돌려준다.
로봇 좌표로 옮기려면 별도의 외부 파라미터(정합)가 필요하다.

사용:
    from astra import Astra
    with Astra() as cam:
        depth_mm = cam.depth()            # (H, W) int16, 0 = 측정 실패
        x, y, z = cam.point(cx, cy)       # 픽셀 → 카메라 좌표 [m]
"""
import ctypes as C
import os
import pathlib
import time

import numpy as np

SDK = pathlib.Path(os.environ.get('ASTRA_SDK', pathlib.Path.home() / 'AstraSDK'))
LIB = SDK / 'lib'


class _Meta(C.Structure):
    _fields_ = [('width', C.c_uint32), ('height', C.c_uint32),
                ('pixelFormat', C.c_int)]


class _Dispatch:
    """여러 .so 에 흩어진 C 심볼을 이름으로 찾아 주는 얇은 래퍼."""

    def __init__(self, libs):
        self._libs, self._cache = libs, {}

    def __getattr__(self, name):
        fn = self._cache.get(name)
        if fn is None:
            for lib in self._libs:
                try:
                    fn = getattr(lib, name)
                    break
                except AttributeError:
                    continue
            else:
                raise AttributeError(f'어느 라이브러리에도 없는 심볼: {name}')
            self._cache[name] = fn
        return fn


class Astra:
    def __init__(self, timeout_ms=2000):
        # 심볼이 두 라이브러리에 나뉘어 있다 — 세션·리더는 libastra_core.so,
        # 프레임·스트림 접근은 libastra.so. 어느 쪽에 있는지 찾아 부른다.
        self._libs = [C.CDLL(str(LIB / n), mode=C.RTLD_GLOBAL)
                      for n in ('libastra_core.so', 'libastra_core_api.so', 'libastra.so')]
        a = self._lib = _Dispatch(self._libs)
        a.astra_initialize()
        self._sensor = C.c_void_p()
        if a.astra_streamset_open(b'device/default', C.byref(self._sensor)) != 0:
            raise RuntimeError('Astra 를 열지 못했습니다 — USB 연결과 udev 규칙을 확인하세요')
        self._reader = C.c_void_p()
        a.astra_reader_create(self._sensor, C.byref(self._reader))
        self._stream = C.c_void_p()
        a.astra_reader_get_depthstream(self._reader, C.byref(self._stream))
        hf, vf = C.c_float(), C.c_float()
        a.astra_depthstream_get_hfov(self._stream, C.byref(hf))
        a.astra_depthstream_get_vfov(self._stream, C.byref(vf))
        self.hfov, self.vfov = hf.value, vf.value
        a.astra_stream_start(self._stream)
        self._shape = None
        # 첫 프레임은 몇 번의 update 뒤에 온다(실측 3회) — 여기서 shape 도 확정된다
        t0 = time.monotonic()
        while True:
            d = self._try_frame()
            if d is not None:
                break
            if (time.monotonic() - t0) * 1000 > timeout_ms:
                raise RuntimeError('깊이 프레임이 오지 않습니다 — 다른 프로세스가 '
                                   '카메라를 쓰고 있는지 확인하세요')
            time.sleep(0.03)

    # -- 프레임 --
    def depth(self, wait_ms=1500):
        """최신 깊이 프레임 (H, W) int16 [mm].

        폴링 방식이라 매 호출에 프레임이 준비돼 있지는 않다(실측: 첫 획득까지 3회
        update 필요). wait_ms 동안 재시도하고, 그래도 없으면 None 을 돌려준다.
        """
        t0 = time.monotonic()
        while True:
            d = self._try_frame()
            if d is not None or (time.monotonic() - t0) * 1000 > wait_ms:
                return d
            time.sleep(0.02)

    def _try_frame(self):
        a = self._lib
        a.astra_update()
        frame = C.c_void_p()
        if a.astra_reader_open_frame(self._reader, 0, C.byref(frame)) != 0:
            return None
        try:
            df = C.c_void_p()
            a.astra_frame_get_depthframe(frame, C.byref(df))
            meta = _Meta()
            a.astra_depthframe_get_metadata(df, C.byref(meta))
            n = C.c_uint32()
            a.astra_depthframe_get_data_byte_length(df, C.byref(n))
            buf = (C.c_int16 * (n.value // 2))()
            a.astra_depthframe_copy_data(df, buf)
            self._shape = (meta.height, meta.width)
            return np.ctypeslib.as_array(buf).reshape(self._shape).copy()
        finally:
            a.astra_reader_close_frame(C.byref(frame))

    # -- 기하 --
    @property
    def shape(self):
        return self._shape

    def point(self, u, v, depth=None):
        """픽셀 (u, v) → 카메라 좌표 (X, Y, Z) [m]. 깊이가 0이면 None.

        내부 파라미터를 따로 안 주므로 FoV 로 초점거리를 역산한다 —
        fx = (W/2) / tan(hfov/2). 공장 캘리브 값이라 mm 급 정밀도에는 부족하지만
        물체 위치를 잡는 용도에는 충분하다.
        """
        import math
        d = self.depth() if depth is None else depth
        if d is None:
            return None
        h, w = d.shape
        z_mm = int(d[int(v), int(u)])
        if z_mm == 0:
            return None
        fx = (w / 2) / math.tan(self.hfov / 2)
        fy = (h / 2) / math.tan(self.vfov / 2)
        z = z_mm / 1000.0
        return ((u - w / 2) * z / fx, (v - h / 2) * z / fy, z)

    def close(self):
        # 종료 순서를 지켜도 SDK 가 코어를 뱉는 일이 있어(18.04 바이너리) 각각 감싼다.
        # 프로세스가 끝나면 커널이 USB 를 회수하므로 실패해도 다음 실행에 지장이 없다.
        for call in (lambda: self._lib.astra_reader_destroy(C.byref(self._reader)),
                     lambda: self._lib.astra_streamset_close(C.byref(self._sensor)),
                     lambda: self._lib.astra_terminate()):
            try:
                call()
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


if __name__ == '__main__':
    with Astra() as cam:
        d = cam.depth()
        h, w = d.shape
        valid = int((d > 0).sum())
        print(f'해상도 {w}x{h} · FoV {np.degrees(cam.hfov):.1f}°x{np.degrees(cam.vfov):.1f}°')
        print(f'유효 픽셀 {valid}/{d.size} ({100*valid/d.size:.1f}%)')
        print(f'중앙 깊이 {d[h//2, w//2]} mm')
        nz = d[d > 0]
        if nz.size:
            print(f'범위 {nz.min()}~{nz.max()} mm · 중앙값 {int(np.median(nz))} mm')
        p = cam.point(w // 2, h // 2, d)
        if p:
            print(f'중앙 픽셀 → 카메라 좌표 ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:.3f}) m')
