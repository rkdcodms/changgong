"""Read-only HTTP presentation for the vision control process."""
from __future__ import annotations
from collections import deque
from pathlib import Path
import ipaddress
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

PAGE = Path(__file__).with_name('preview.html').read_bytes()
TAILSCALE_NETWORK = ipaddress.IPv4Network('100.64.0.0/10')


def client_is_allowed(address, allowed_clients, allowed_networks=()):
    """Check the socket peer, never a forwarded header supplied by the client."""
    try:
        peer = ipaddress.IPv4Address(address)
    except (ipaddress.AddressValueError, ValueError):
        return False
    return str(peer) in allowed_clients or any(peer in network for network in allowed_networks)

class FrameHub:
    def __init__(self, max_seconds=0, stale_seconds=2.0, clock=None):
        self.clock = clock or time.monotonic
        self.started_at = self.clock()
        self.max_seconds = max_seconds
        self.stale_seconds = stale_seconds
        self.condition = threading.Condition()
        self.stopped = threading.Event()
        self.jpeg = None
        self.sequence = 0
        self.captured_at = None
        self.width = self.height = 0
        self.error = None
        self.frame_times = deque(maxlen=30)

    def publish(self, jpeg, width, height, captured_at=None):
        with self.condition:
            self.jpeg = bytes(jpeg)
            self.sequence += 1
            self.captured_at = self.clock() if captured_at is None else captured_at
            self.frame_times.append(self.captured_at)
            self.width, self.height = width, height
            self.error = None
            self.condition.notify_all()

    def fail(self, message):
        with self.condition:
            self.error = str(message)
            self.condition.notify_all()

    def stop(self):
        self.stopped.set()
        with self.condition:
            self.condition.notify_all()

    def _status_locked(self):
        now = self.clock()
        age = None if self.captured_at is None else max(0.0, now - self.captured_at)
        span = self.frame_times[-1] - self.frame_times[0] if len(self.frame_times) > 1 else 0
        healthy = self.jpeg is not None and age <= self.stale_seconds and self.error is None and not self.stopped.is_set()
        error = self.error
        if self.stopped.is_set():
            error = '카메라 미리보기가 종료되었습니다.'
        elif self.jpeg is not None and age > self.stale_seconds and error is None:
            error = '카메라에서 새 영상이 들어오지 않습니다.'
        return {'ok': healthy, 'frame_sequence': self.sequence, 'capture_age_seconds': age,
                'fps': (len(self.frame_times) - 1) / span if span > 0 else 0.0,
                'width': self.width, 'height': self.height, 'error': error,
                'unlimited': self.max_seconds == 0,
                'remaining_seconds': None if self.max_seconds == 0 else max(0.0, self.max_seconds - (now - self.started_at))}

    def status(self):
        with self.condition:
            return self._status_locked()

    def next_frame(self, after_sequence, timeout=1.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while not self.stopped.is_set():
                state = self._status_locked()
                if state['ok'] and self.sequence > after_sequence:
                    return self.sequence, self.jpeg
                if self.error is not None:
                    return None
                wait = deadline - time.monotonic()
                if wait <= 0:
                    return None
                self.condition.wait(wait)
        return None


def multipart_frame(jpeg):
    return b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode('ascii') + b'\r\n\r\n' + jpeg + b'\r\n'


class PreviewHandler(BaseHTTPRequestHandler):
    server_version = 'JetsonVision'
    sys_version = ''
    protocol_version = 'HTTP/1.1'

    def log_message(self, format, *args):
        # Avoid noisy per-second requests; startup and failures are logged by main.
        pass

    def _headers(self, status, content_type, length=None):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Cross-Origin-Resource-Policy', 'same-origin')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Content-Security-Policy', "default-src 'none'; img-src 'self' blob: data:; connect-src 'self'; style-src 'unsafe-inline'; script-src 'unsafe-inline'; frame-ancestors 'none'; base-uri 'none'")
        if length is not None:
            self.send_header('Content-Length', str(length))
        else:
            self.send_header('Connection', 'close')
            self.close_connection = True
        self.end_headers()

    def _send(self, status, content_type, body):
        self._headers(status, content_type, len(body))
        self.wfile.write(body)

    def do_GET(self):
        self.connection.settimeout(5)
        try:
            if not self.server.is_client_allowed(self.client_address[0]):
                self.close_connection = True
                self._send(403, 'text/plain; charset=utf-8', '이 컴퓨터에는 카메라 보기 권한이 없습니다.'.encode('utf-8'))
                return
            path = urlsplit(self.path).path
            if path == '/':
                self._send(200, 'text/html; charset=utf-8', PAGE)
            elif path == '/status.json':
                data = json.dumps(self.server.hub.status(), ensure_ascii=False, allow_nan=False).encode('utf-8')
                self._send(200, 'application/json; charset=utf-8', data)
            elif path == '/snapshot.jpg':
                frame = self.server.hub.next_frame(0, timeout=0)
                if frame is None:
                    self._send(503, 'text/plain; charset=utf-8', '새 카메라 영상이 없습니다.'.encode('utf-8'))
                else:
                    self._send(200, 'image/jpeg', frame[1])
            elif path == '/stream.mjpg':
                self._stream()
            else:
                self._send(404, 'text/plain; charset=utf-8', b'Not found')
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            self.close_connection = True

    def _stream(self):
        if not self.server.stream_slots.acquire(blocking=False):
            self._send(503, 'text/plain; charset=utf-8', '동시 접속 화면이 너무 많습니다.'.encode('utf-8'))
            return
        try:
            frame = self.server.hub.next_frame(0, timeout=3)
            if frame is None:
                self._send(503, 'text/plain; charset=utf-8', '카메라 준비 상태를 확인해 주세요.'.encode('utf-8'))
                return
            self._headers(200, 'multipart/x-mixed-replace; boundary=frame')
            while frame is not None and not self.server.hub.stopped.is_set():
                sequence, jpeg = frame
                self.wfile.write(multipart_frame(jpeg))
                self.wfile.flush()
                frame = self.server.hub.next_frame(sequence, timeout=2)
        finally:
            self.server.stream_slots.release()


class PreviewServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    allow_reuse_address = True

    def __init__(self, address, hub, allowed_clients=(), allowed_networks=()):
        self.hub = hub
        self.allowed_clients = frozenset(str(ipaddress.IPv4Address(value)) for value in allowed_clients)
        self.allowed_networks = tuple(ipaddress.IPv4Network(value) for value in allowed_networks)
        if any(not network.subnet_of(TAILSCALE_NETWORK) for network in self.allowed_networks):
            raise ValueError('Allowed networks must be inside the Tailscale IPv4 range')
        if not self.allowed_clients and not self.allowed_networks:
            raise ValueError('A viewer access policy is required')
        self.stream_slots = threading.BoundedSemaphore(4)
        super().__init__(address, PreviewHandler)

    def is_client_allowed(self, address):
        return client_is_allowed(address, self.allowed_clients, self.allowed_networks)


