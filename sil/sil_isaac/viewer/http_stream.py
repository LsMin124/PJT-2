"""TCP 전용 관전 경로 — 뷰포트를 HTTP MJPEG로 스트리밍.

UDP 차단망(캠퍼스 내부망) 대응: tailscale이 DERP(TCP/443) 릴레이로 떨어져도
HTTP는 그대로 통과한다. WebRTC(UDP 터널링) 스트림이 끊기는 환경의 대체 경로.

사용 (kit 앱 안에서):
    from http_stream import HttpViewer
    viewer = HttpViewer(port=8211)          # 데몬 스레드로 서버 기동
    while app.is_running():
        app.update()
        viewer.tick()                        # 주기 캡처 (in-flight 1건)

접속: http://<서버IP>:8211/  (브라우저 MJPEG) · /frame (단장 JPEG)
"""

import ctypes
import io
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CAPTURE_INTERVAL = 0.15  # 초 — 약 6fps 상한 (DERP 릴레이 대역폭 고려)
JPEG_QUALITY = 70
BOUNDARY = b"--isaacframe"

INDEX_HTML = b"""<!doctype html><meta charset="utf-8">
<title>T3 viewer (TCP)</title>
<body style="margin:0;background:#111;display:grid;place-items:center;height:100vh">
<img src="/stream" style="max-width:100vw;max-height:100vh">
"""


class _FrameHub:
    """최신 JPEG 1장만 보관 — 느린 클라이언트가 밀려도 메모리 고정."""

    def __init__(self):
        self._jpeg = None
        self._seq = 0
        self._cond = threading.Condition()

    def push(self, jpeg: bytes) -> None:
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    def wait_next(self, last_seq: int, timeout: float = 2.0):
        with self._cond:
            if self._seq == last_seq:
                self._cond.wait(timeout)
            return self._jpeg, self._seq


class _Handler(BaseHTTPRequestHandler):
    hub: _FrameHub = None  # 서버 기동 시 주입

    def log_message(self, *a):  # 접속 로그로 kit 콘솔 오염 방지
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(INDEX_HTML)))
            self.end_headers()
            self.wfile.write(INDEX_HTML)
        elif self.path.startswith("/frame"):
            jpeg, _ = self.hub.wait_next(-1)
            if jpeg is None:
                self.send_error(503, "no frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpeg)
        elif self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header(
                "Content-Type",
                f"multipart/x-mixed-replace; boundary={BOUNDARY.decode()[2:]}")
            self.end_headers()
            seq = -1
            try:
                while True:
                    jpeg, seq2 = self.hub.wait_next(seq)
                    if jpeg is None or seq2 == seq:
                        continue
                    seq = seq2
                    self.wfile.write(
                        BOUNDARY + b"\r\nContent-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # 클라이언트 이탈은 정상 경로
        else:
            self.send_error(404)


class HttpViewer:
    def __init__(self, port: int = 8211):
        from omni.kit.viewport.utility import (capture_viewport_to_buffer,
                                               get_active_viewport)
        from PIL import Image

        self._capture = capture_viewport_to_buffer
        self._viewport = get_active_viewport()
        self._Image = Image
        self.hub = _FrameHub()
        self._inflight = False
        self._last_t = 0.0
        handler = type("H", (_Handler,), {"hub": self.hub})
        self._srv = ThreadingHTTPServer(("0.0.0.0", port), handler)
        threading.Thread(target=self._srv.serve_forever, daemon=True).start()
        print(f"[http_stream] TCP 관전 경로 준비 — http://0.0.0.0:{port}/", flush=True)

    def tick(self) -> None:
        """kit 메인 루프에서 매 프레임 호출 — 주기 도달 시 캡처 1건 발행."""
        now = time.monotonic()
        if self._inflight or now - self._last_t < CAPTURE_INTERVAL:
            return
        self._inflight = True
        self._last_t = now
        self._capture(self._viewport, self._on_capture)

    def _on_capture(self, buffer, buffer_size, width, height, fmt):
        try:
            # PyCapsule → 바이트 (omni 뷰포트 버퍼 캡처 표준 경로, RGBA8)
            gp = ctypes.pythonapi.PyCapsule_GetPointer
            gp.restype = ctypes.c_void_p
            gp.argtypes = [ctypes.py_object, ctypes.c_char_p]
            ptr = gp(buffer, None)
            if not ptr or buffer_size != width * height * 4:
                return
            raw = bytes((ctypes.c_ubyte * buffer_size).from_address(ptr))
            img = self._Image.frombytes("RGBA", (width, height), raw).convert("RGB")
            out = io.BytesIO()
            img.save(out, "JPEG", quality=JPEG_QUALITY)
            self.hub.push(out.getvalue())
        except Exception as e:  # 캡처 1건 실패가 루프를 죽이면 안 됨
            print(f"[http_stream] capture drop: {e}", flush=True)
        finally:
            self._inflight = False
