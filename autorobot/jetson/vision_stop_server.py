"""Live YOLO view plus VC1 vision-stop control. Start only with the VC1 Mega sketch.

Opening the serial port may RESET the Mega. The VC1 sketch must keep motors
disabled until a healthy inference/serial heartbeat handshake has completed.
"""
from __future__ import annotations

import argparse
from collections import deque
import datetime
import hashlib
import ipaddress
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import secrets
import signal
import socket
import threading
import time

from vision_control import BoundedLines, NAMES, VisionStopController
from web_preview import FrameHub, PreviewServer, TAILSCALE_NETWORK

VERSION = 'vision-stop-v1'
DEFAULT_WEIGHTS = Path(__file__).resolve().parents[1] / 'models' / 'best.pt'
WEIGHTS_HASH = 'e67df5f04399513f8578a6948843902ea31b346c558375af88e919834283bd84'


class VisionHub(FrameHub):
    def __init__(self, controller, max_seconds=0, rotation_ccw=0, stop_zone=(.4, .6)):
        super().__init__(max_seconds=max_seconds, stale_seconds=1.0)
        self.controller = controller
        self.detections = []
        self.inference_times = deque(maxlen=30)
        self.inference_ms = None
        self.model_error = None
        self.camera_open = False
        self.rotation_ccw = rotation_ccw
        self.stop_zone = stop_zone
        self.sensor_values = None
        self.sensor_at = None

    def record_sensors(self, values, now):
        with self.condition:
            self.sensor_values = list(values)
            self.sensor_at = now

    def inference(self, detections, elapsed_ms):
        with self.condition:
            self.detections = detections
            self.inference_ms = elapsed_ms
            self.inference_times.append(time.monotonic())

    def status(self):
        with self.condition:
            state = self._status_locked()
            whites = [v <= 500 for v in self.sensor_values] if self.sensor_values is not None else None
            state.update(sensor_values=list(self.sensor_values) if self.sensor_values is not None else None,
                         white_count=sum(whites) if whites is not None else None,
                         normal_line=((whites[3] or whites[4]) and not whites[0] and not whites[7]) if whites is not None else None,
                         sensor_age_seconds=max(0., time.monotonic()-self.sensor_at) if self.sensor_at is not None else None)
            span = self.inference_times[-1] - self.inference_times[0] if len(self.inference_times) > 1 else 0
            state.update(version=VERSION, detections=list(self.detections), inference_ms=self.inference_ms,
                         inference_fps=(len(self.inference_times)-1)/span if span > 0 else 0.,
                         camera_open=self.camera_open, model_error=self.model_error, rotation_ccw=self.rotation_ccw,
                         stop_zone={'left': self.stop_zone[0], 'right': self.stop_zone[1]},
                         stop_confidence_threshold=self.controller.confidence,
                         stop_min_span_seconds=self.controller.min_span,
                         stop_min_frames=self.controller.min_frames,
                         confidence_label='신뢰도', confidence_is_measured_accuracy=False)
        control = self.controller.status(time.monotonic())
        state.update(control=control, model_ready=control['model_ready'],
                     serial_ready=control['serial_connected'] and control['serial_fresh'],
                     heartbeat_allowed=control['heartbeat_allowed'], mega_mode=control['mega_mode'],
                     stop_reason=control['stop_reason'], fault=control['fatal'],
                     control_ready=control['heartbeat_allowed'] and control['mega_mode'] not in ('FAULT', 'WAIT_VISION', 'UNSEEN'))
        return state


def parse_sensor_line(line):
    """Read the existing eight-ADC tab telemetry; do not change robot control."""
    if not isinstance(line, bytes) or len(line) > 48:
        return None
    parts = line.rstrip(b'\r\n').split(b'\t')
    if len(parts) != 8 or any(not 1 <= len(p) <= 4 or not p.isdigit() for p in parts):
        return None
    values = [int(p) for p in parts]
    return values if all(0 <= v <= 1023 for v in values) else None


def drain_startup_serial(link, stopped, clock=None):
    """Discard boot/reset residue before any telemetry processing or heartbeat."""
    clock = clock or time.monotonic
    deadline = clock() + 1.5
    while clock() < deadline:
        if stopped.is_set():
            return False
        link.read(256)  # bounded by Serial(timeout=.02), with no outgoing commands
    if stopped.is_set():
        return False
    link.reset_input_buffer()
    return True


def choose_port(value):
    if value != 'auto':
        return value
    from serial.tools import list_ports
    candidates = [p.device for p in list_ports.comports() if p.vid in (0x2341, 0x2a03, 0x1a86) and p.device.startswith('/dev/')]
    if len(candidates) != 1:
        raise RuntimeError('Exactly one Mega/CH340 port is required when --serial auto is used')
    return candidates[0]


class SerialWorker(threading.Thread):
    def __init__(self, core, hub, port, logger):
        super().__init__(name='mega-vision-link', daemon=True)
        self.core, self.hub, self.port, self.logger = core, hub, port, logger

    def run(self):
        link = None
        try:
            import serial
            link = serial.Serial(port=None, baudrate=115200, timeout=.02, write_timeout=.1, exclusive=True)
            link.dtr = False
            link.rts = False
            link.port = choose_port(self.port)
            link.open()
            if not drain_startup_serial(link, self.hub.stopped):
                return
            self.core.set_serial_connected(True, time.monotonic())
            framing = BoundedLines()
            while not self.hub.stopped.is_set():
                for line in framing.feed(link.read(256)):
                    now = time.monotonic()
                    values = parse_sensor_line(line)
                    if values is not None:
                        self.hub.record_sensors(values, now)
                    else:
                        self.core.ingest_line(line, now)
                for command in self.core.outgoing(time.monotonic()):
                    if link.write(command) != len(command):
                        raise OSError('Incomplete serial command write')
                for event in self.core.drain_events():
                    self.logger.info(json.dumps(event, ensure_ascii=False, allow_nan=False))
        except Exception as exc:
            self.core.set_serial_connected(False, time.monotonic(), str(exc))
            self.logger.error(json.dumps({'event': 'serial_error', 'error': str(exc)}))
        finally:
            if link is not None:
                link.close()


class Detector(threading.Thread):
    def __init__(self, core, hub, weights, camera, logger, rotation_ccw=0, stop_zone=(.4, .6)):
        super().__init__(name='yolo-camera-control', daemon=True)
        self.core, self.hub, self.weights, self.camera, self.logger = core, hub, weights, camera, logger
        self.rotation_ccw = rotation_ccw
        self.stop_zone = stop_zone

    def run(self):
        cap = gpu_lock = None
        try:
            if hashlib.sha256(self.weights.read_bytes()).hexdigest() != WEIGHTS_HASH:
                raise RuntimeError('Model checksum differs from the verified statue model')
            gpu_lock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            gpu_lock.bind('\0hw2-gpu0')
            import cv2
            import numpy as np
            import torch
            from ultralytics import YOLO
            if not torch.cuda.is_available():
                raise RuntimeError('CUDA is unavailable; vision control remains disabled')
            torch.set_num_threads(2)
            model = YOLO(str(self.weights))
            if model.names != NAMES:
                raise RuntimeError('Model class mapping differs from the verified 8-class mapping')
            model.predict(np.zeros((640, 640, 3), dtype=np.uint8), imgsz=640, device='cuda:0', verbose=False, save=False)
            cap = cv2.VideoCapture(int(self.camera) if self.camera.isdigit() else self.camera, cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError('Camera is unavailable or in use by another program')
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.hub.camera_open = True
            self.core.set_model_ready(True, time.monotonic())
            last_preview = 0.
            while not self.hub.stopped.is_set():
                # Host acquisition time is used; this camera API provides no sensor exposure timestamp.
                captured = time.monotonic()
                ok, frame = cap.read()
                if not ok or frame is None:
                    raise RuntimeError('Camera frame read failed; heartbeat has been stopped')
                frame = rotate_input(frame, self.rotation_ccw, cv2)
                infer_started = time.monotonic()
                result = model.predict(frame, imgsz=640, device='cuda:0', conf=.25,
                                       max_det=20, verbose=False, save=False)[0]
                finished = time.monotonic()
                detections = [{'class_id': int(box.cls.item()), 'class_name': NAMES[int(box.cls.item())],
                               'confidence': float(box.conf.item()), 'bbox': [float(v) for v in box.xyxy[0].tolist()]}
                              for box in result.boxes]
                for item in detections:
                    item['stop_eligible'] = stop_eligible(item, frame.shape[1], self.stop_zone, self.core.confidence)
                self.core.observe(detections, captured, finished)
                self.hub.inference(detections, (finished-infer_started)*1000)
                if finished-last_preview >= .1:
                    annotated = frame.copy()
                    height, width = annotated.shape[:2]
                    for edge in self.stop_zone:
                        x = int(width*edge)
                        cv2.line(annotated, (x, 0), (x, height-1), (0, 220, 255), 2)
                    for item in detections:
                        x1, y1, x2, y2 = [int(v) for v in item['bbox']]
                        color = (70, 230, 100) if item['stop_eligible'] else (190, 165, 120)
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(annotated, f"{item['class_name']} {item['confidence']*100:.0f}%",
                                    (max(0, x1), max(18, y1-7)), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
                    state = self.core.status(finished)
                    cv2.putText(annotated, 'VISION STOP / Mega: '+state['mega_mode'], (8, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 210, 255), 2)
                    cv2.putText(annotated, f'STOP ZONE {self.stop_zone[0]*100:.0f}-{self.stop_zone[1]*100:.0f}%', (8, 44),
                                cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 210, 255), 1)
                    encoded, jpeg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if not encoded:
                        raise RuntimeError('Preview JPEG encoding failed')
                    self.hub.publish(jpeg.tobytes(), annotated.shape[1], annotated.shape[0], captured_at=captured)
                    last_preview = finished
        except Exception as exc:
            self.hub.model_error = str(exc)
            self.hub.fail(str(exc))
            self.core.fail('VISION_ERROR: '+str(exc), time.monotonic())
            self.core.set_model_ready(False, time.monotonic())
            self.logger.error(json.dumps({'event': 'vision_error', 'error': str(exc)}))
        finally:
            self.hub.camera_open = False
            if cap is not None:
                cap.release()
            if gpu_lock is not None:
                gpu_lock.close()


def stop_eligible(item, width, stop_zone, threshold):
    center = (item['bbox'][0] + item['bbox'][2]) / (2*width)
    return bool(item['confidence'] >= threshold and stop_zone[0] <= center <= stop_zone[1])


def rotate_input(frame, degrees, cv2):
    """Rotate the actual detector input and preview together, never just CSS."""
    if degrees == 0:
        return frame
    code = {90: cv2.ROTATE_90_COUNTERCLOCKWISE, 180: cv2.ROTATE_180,
            270: cv2.ROTATE_90_CLOCKWISE}[degrees]
    return cv2.rotate(frame, code)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bind', required=True)
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--allow-client', action='append', default=[],
                    help='Allow this viewer IPv4 address; may be repeated')
    ap.add_argument('--allow-tailscale', action='store_true',
                    help='Allow every Tailscale IPv4 peer that can reach this device')
    ap.add_argument('--camera', default='/dev/video0')
    ap.add_argument('--rotate-ccw', type=int, choices=(0, 90, 180, 270), default=0)
    ap.add_argument('--stop-zone-left', type=float, default=.4)
    ap.add_argument('--stop-zone-right', type=float, default=.6)
    ap.add_argument('--serial', required=True,
                    help='Mega serial device path, or auto when exactly one candidate exists')
    ap.add_argument('--weights', type=Path, default=DEFAULT_WEIGHTS)
    ap.add_argument('--confidence', type=float, default=.55)
    ap.add_argument('--max-seconds', type=int, default=0)
    ap.add_argument('--status-file', type=Path)
    ap.add_argument('--log-file', type=Path, default=Path(__file__).with_name('vision_events.jsonl'))
    args = ap.parse_args(argv)
    try:
        bind = ipaddress.IPv4Address(args.bind)
        args.allow_client = [str(ipaddress.IPv4Address(value)) for value in args.allow_client]
    except (ipaddress.AddressValueError, ValueError):
        ap.error('Bind and explicit viewers must be valid IPv4 addresses')
    if not (bind in TAILSCALE_NETWORK or bind.is_loopback):
        ap.error('Bind only a specific Tailscale or loopback IPv4 address')
    if not args.allow_tailscale and not args.allow_client:
        ap.error('Specify --allow-tailscale or at least one --allow-client')
    if args.allow_tailscale and bind not in TAILSCALE_NETWORK:
        ap.error('--allow-tailscale requires binding the specific Tailscale IPv4 address')
    if not 0 < args.confidence <= 1 or args.max_seconds < 0 or not 1 <= args.port <= 65535:
        ap.error('Invalid confidence or lifetime')
    if not 0 <= args.stop_zone_left < args.stop_zone_right <= 1:
        ap.error('Stop zone must satisfy 0 <= left < right <= 1')
    args.bind = str(bind)
    return args


def main(argv=None):
    args = parse_args(argv)
    bind = args.bind
    args.log_file.parent.mkdir(parents=True, exist_ok=True)
    settings = args.log_file.parent/'yolo_settings'
    (settings/'Ultralytics').mkdir(parents=True, exist_ok=True)
    os.environ.update(YOLO_CONFIG_DIR=str(settings), YOLO_AUTOINSTALL='false', YOLO_OFFLINE='true', MPLBACKEND='Agg')
    logger = logging.getLogger(VERSION)
    logger.setLevel(logging.INFO)
    handler = RotatingFileHandler(args.log_file, maxBytes=1_000_000, backupCount=2, encoding='utf-8')
    handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    logger.addHandler(handler)
    core = VisionStopController(session=secrets.randbelow(0xffffffff)+1, confidence=args.confidence,
                                min_span=.1, min_frames=3)
    stop_zone = (args.stop_zone_left, args.stop_zone_right)
    hub = VisionHub(core, args.max_seconds, args.rotate_ccw, stop_zone)
    server = PreviewServer((bind, args.port), hub, args.allow_client,
                           allowed_networks=(TAILSCALE_NETWORK,) if args.allow_tailscale else ())
    server.timeout = .1
    serial = SerialWorker(core, hub, args.serial, logger)
    detector = Detector(core, hub, args.weights, args.camera, logger, args.rotate_ccw, stop_zone)
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: hub.stop())
    serial.start()
    detector.start()
    last_status = 0.
    logger.info(json.dumps({'event': 'started', 'version': VERSION, 'session': core.session, 'url': f'http://{bind}:{args.port}/'}))
    try:
        while not hub.stopped.is_set():
            server.handle_request()
            now = time.monotonic()
            if now-last_status >= .25:
                state = hub.status()
                state['updated_at_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
                if args.status_file:
                    args.status_file.parent.mkdir(parents=True, exist_ok=True)
                    tmp = args.status_file.with_name(args.status_file.name+'.tmp')
                    tmp.write_text(json.dumps(state, ensure_ascii=False, allow_nan=False), encoding='utf-8')
                    os.replace(tmp, args.status_file)
                last_status = now
                if state['remaining_seconds'] is not None and state['remaining_seconds'] <= 0:
                    break
    finally:
        hub.stop()
        core.fail('PROGRAM_STOPPED', time.monotonic())
        server.server_close()
        serial.join(timeout=2)
        detector.join(timeout=3)
        logger.info(json.dumps({'event': 'stopped', 'serial_thread_exited': not serial.is_alive(), 'detector_thread_exited': not detector.is_alive()}))
        handler.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
