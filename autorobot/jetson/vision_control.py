"""Pure VC1 stop protocol and recognition consensus; no hardware access here."""
from __future__ import annotations

from collections import deque
import math
import threading

NAMES = {0: 'Calypso', 1: 'Cattle', 2: 'Cyclopes', 3: 'Laestrygonians',
         4: 'Poseidon', 5: 'Scylla', 6: 'Siren', 7: 'TrojanHorse'}
MODES = {0: 'FOLLOW', 1: 'STOPPED', 2: 'EXIT_BAR', 3: 'SEARCH', 4: 'FAULT', 5: 'WAIT_VISION'}
REASONS = {0: 'NONE', 1: 'WHITE_LINE', 2: 'VISION', 3: 'LINK_FAULT'}
U32 = 0xffffffff


class BoundedLines:
    def __init__(self, limit=96):
        self.limit = limit
        self.data = bytearray()
        self.discard = False

    def feed(self, chunk):
        rows = []
        for byte in chunk:
            if byte == 10:
                if not self.discard:
                    rows.append(bytes(self.data) + b'\n')
                self.data.clear()
                self.discard = False
            elif not self.discard:
                if len(self.data) >= self.limit - 1:
                    self.data.clear()
                    self.discard = True
                else:
                    self.data.append(byte)
        return rows


def parse_line(line):
    if not isinstance(line, bytes) or len(line) > 96:
        return None
    parts = line.rstrip(b'\r\n').split(b',')
    if len(parts) != 6 or parts[0] not in (b'@LT1', b'@V1'):
        return None
    values = []
    for index, part in enumerate(parts[1:]):
        if parts[0] == b'@V1' and index == 2 and part == b'-1':
            values.append(-1)
        elif 1 <= len(part) <= 10 and part.isdigit() and int(part) <= U32:
            values.append(int(part))
        else:
            return None
    if parts[0] == b'@LT1':
        mode, seq, age, uptime, test = values
        if mode not in MODES or test not in (0, 1):
            return None
        return 'lt', dict(mode=mode, seq=seq, age=age, uptime=uptime, test=test)
    session, event, label, reason, live = values
    if label not in (-1, *NAMES) or reason not in REASONS or live not in (0, 1):
        return None
    return 'v1', dict(session=session, event=event, label=label, reason=reason, live=live)


def _same_object(previous, current):
    """Conservative adjacent-frame association, not a persistent identity claim.

    Nearby overlapping boxes with comparable area may continue one track.
    A disjoint same-class box or a sudden size jump must establish its own
    consensus before it can request a stop.
    """
    old_area = (previous[2] - previous[0]) * (previous[3] - previous[1])
    new_area = (current[2] - current[0]) * (current[3] - current[1])
    if not math.isfinite(old_area) or not math.isfinite(new_area) or old_area <= 0 or new_area <= 0:
        return False
    if not .5 <= new_area / old_area <= 2.:
        return False
    width = max(0., min(previous[2], current[2]) - max(previous[0], current[0]))
    height = max(0., min(previous[3], current[3]) - max(previous[1], current[1]))
    intersection = width * height
    return intersection / (old_area + new_area - intersection) >= .15


class VisionStopController:
    """A single owner calls outgoing(); all public methods are thread-safe.

    Model/serial errors and missing stop acknowledgement latch a host fault.
    Recover those by restarting this program and RESETting the Mega. Ordinary
    stale frames suppress heartbeats; Mega's own 1500 ms watchdog provides stop.
    """
    def __init__(self, session, confidence=.55, min_frames=3, min_span=.2,
                 absence=1.5, restart_cooldown=2., frame_freshness=1.,
                 serial_freshness=.75, continuity_gap=.35):
        if not isinstance(session, int) or not 1 <= session <= U32:
            raise ValueError('session must be a nonzero uint32')
        self.session = session
        self.confidence, self.min_frames, self.min_span = confidence, min_frames, min_span
        self.absence, self.restart_cooldown = absence, restart_cooldown
        self.frame_freshness, self.serial_freshness, self.continuity_gap = frame_freshness, serial_freshness, continuity_gap
        self.lock = threading.RLock()
        self.model_ready = self.serial_connected = False
        self.fatal = None
        self.lt = self.v1 = None
        self.lt_at = self.v1_at = None
        self.capture_at = self.inference_at = None
        self.last_heartbeat = float('-inf')
        self.last_outgoing = None
        self.last_resume = None
        self.next_event = 1
        self.pending = None
        self.last_ack = None
        self.last_request = None
        self.streaks = {}
        self.latched = set()
        self.absent_since = {}
        self.visible = {}
        self.eligible = {}
        self.events = deque(maxlen=128)
        self.history = deque(maxlen=32)
        self.total_stop_requests = self.total_accepted_stops = 0

    def _event(self, kind, now, **fields):
        event = {'event': kind, 'at': now, **fields}
        self.events.append(event)
        self.history.append(event)

    def _fault(self, message, now):
        if self.fatal is None:
            self.fatal = message
            self._event('host_fault', now, reason=message)

    def set_model_ready(self, ready, now):
        with self.lock:
            self.model_ready = bool(ready)
            if not ready and self.inference_at is not None:
                self._fault('MODEL_OR_CAMERA_FAILED', now)

    def set_serial_connected(self, connected, now, error=None):
        with self.lock:
            self.serial_connected = bool(connected)
            if not connected and (error or self.lt is not None):
                self._fault('SERIAL_DISCONNECTED' + (': ' + str(error) if error else ''), now)

    def fail(self, reason, now):
        with self.lock:
            self._fault(str(reason), now)

    def ingest_line(self, line, now):
        parsed = parse_line(line)
        if parsed is None:
            return False
        with self.lock:
            kind, value = parsed
            if kind == 'lt':
                if self.lt_at is not None and now < self.lt_at:
                    return False
                old = self.lt
                if old and ((value['uptime'] - old['uptime']) & U32) >= (1 << 31):
                    self._fault('MEGA_RESET_DETECTED_RESTART_APP', now)
                if old and old['mode'] == 1 and value['mode'] in (0, 2, 3):
                    self.last_resume = now
                    self._event('mega_resumed', now, stop_seq=value['seq'])
                self.lt, self.lt_at = value, now
                if old is None or any(old[k] != value[k] for k in ('mode', 'seq', 'test')):
                    self._event('mega_state', now, mode=MODES[value['mode']], stop_seq=value['seq'])
            else:
                if self.v1_at is not None and now < self.v1_at:
                    return False
                self.v1, self.v1_at = value, now
                if value['session'] not in (0, self.session):
                    self._fault('SESSION_MISMATCH_RESET_MEGA_AND_RESTART_APP', now)
            self._check_ack(now)
            return True

    def _check_ack(self, now):
        if not self.pending or not self.lt or not self.v1 or self.fatal:
            return
        p, v, lt = self.pending, self.v1, self.lt
        if (now - p['created_at'] >= 1.0):
            self._fault('STOP_ACK_TIMEOUT', now)
            return
        if (v['session'] == self.session and v['event'] == p['event'] and
                v['label'] == p['class_id'] and v['reason'] == 2 and
                lt['mode'] == 1 and lt['seq'] == ((p['baseline_seq'] + 1) & U32) and
                self.lt_at >= p['created_at'] and self.v1_at >= p['created_at'] and
                now - self.lt_at < self.serial_freshness and now - self.v1_at < self.serial_freshness):
            self.last_ack = {'event': p['event'], 'class_id': p['class_id'], 'class_name': NAMES[p['class_id']],
                             'stop_seq': lt['seq'], 'confirmed_at': now}
            self.pending = None
            self.total_accepted_stops += 1
            self._event('stop_confirmed', now, event_id=self.last_ack['event'], class_id=p['class_id'],
                        class_name=NAMES[p['class_id']], stop_seq=lt['seq'])

    def observe(self, detections, captured_at, finished_at):
        with self.lock:
            if not all(math.isfinite(v) for v in (captured_at, finished_at)):
                self._fault('INVALID_FRAME_TIME', finished_at)
                return
            if captured_at > finished_at or finished_at - captured_at >= self.frame_freshness:
                self.streaks.clear()
                self.absent_since.clear()
                return
            if self.capture_at is not None and captured_at <= self.capture_at:
                return
            gap = None if self.capture_at is None else captured_at - self.capture_at
            continuous = gap is not None and gap <= self.continuity_gap
            if not continuous:
                self.streaks.clear()
                self.absent_since.clear()
            self.capture_at, self.inference_at = captured_at, finished_at
            visible = {}
            candidates = {}
            for item in detections:
                cid, conf, box = item.get('class_id'), item.get('confidence'), item.get('bbox')
                if (not isinstance(cid, int) or isinstance(cid, bool) or cid not in NAMES or not isinstance(conf, (int, float)) or
                        not math.isfinite(conf) or not .25 <= conf <= 1 or
                        not isinstance(box, (list, tuple)) or len(box) != 4 or
                        not all(isinstance(v, (int, float)) and math.isfinite(v) for v in box) or
                        box[2] <= box[0] or box[3] <= box[1]):
                    continue
                visible[cid] = max(visible.get(cid, 0), conf)
                if conf >= self.confidence:
                    candidates.setdefault(cid, []).append({
                        'confidence': conf, 'bbox': tuple(box),
                        'eligible': item.get('stop_eligible', True) is True})
            self.visible = visible
            self.eligible = {}
            self.streaks = {cid: value for cid, value in self.streaks.items() if cid in candidates}
            for cid, items in candidates.items():
                old = self.streaks.get(cid)
                matches = [item for item in items if old and _same_object(old['bbox'], item['bbox'])]
                # Prefer a currently aligned object, but never give a new one
                # another instance's accumulated same-class evidence.
                aligned = [item for item in items if item['eligible']]
                if aligned:
                    selected = matches[0] if len(matches) == 1 and matches[0]['eligible'] else max(aligned, key=lambda item: item['confidence'])
                else:
                    selected = matches[0] if len(matches) == 1 else max(items, key=lambda item: item['confidence'])
                if len(matches) == 1 and selected is matches[0]:
                    streak = old
                    streak['count'] += 1
                else:
                    streak = {'count': 1, 'since': captured_at}
                streak['bbox'] = selected['bbox']
                self.streaks[cid] = streak
                if selected['eligible']:
                    self.eligible[cid] = selected['confidence']
            for cid in list(self.latched):
                if cid in visible:
                    self.absent_since.pop(cid, None)
                else:
                    since = self.absent_since.setdefault(cid, captured_at)
                    if captured_at - since >= self.absence:
                        self.latched.remove(cid)
                        self.absent_since.pop(cid, None)
                        self._event('class_rearmed', finished_at, class_id=cid, class_name=NAMES[cid])

    def _healthy(self, now):
        return (self.fatal is None and self.model_ready and self.serial_connected and
                self.capture_at is not None and self.inference_at is not None and
                0 <= now - self.capture_at < self.frame_freshness and
                0 <= now - self.inference_at < self.frame_freshness and
                self.lt_at is not None and self.v1_at is not None and
                0 <= now - self.lt_at < self.serial_freshness and
                0 <= now - self.v1_at < self.serial_freshness and
                self.lt['test'] == 0 and self.v1['session'] in (0, self.session))

    def outgoing(self, now):
        with self.lock:
            commands = []
            if self.pending and now - self.pending['created_at'] >= 1.0:
                self._fault('STOP_ACK_TIMEOUT', now)
            if not self._healthy(now):
                return commands
            if now - self.last_heartbeat >= .2 - 1e-9:
                commands.append(f'@VC1,H,{self.session}\n'.encode('ascii'))
                self.last_heartbeat = now
            can_start = (not self.pending and self.lt['mode'] in (0, 2, 3) and self.v1['session'] == self.session and
                         self.v1['live'] == 1 and (self.last_resume is None or now - self.last_resume >= self.restart_cooldown))
            if can_start:
                choices = [cid for cid, s in self.streaks.items() if cid in self.eligible and cid not in self.latched and
                           s['count'] >= self.min_frames and self.capture_at - s['since'] >= self.min_span - 1e-9]
                if choices:
                    if self.next_event > U32:
                        self._fault('EVENT_COUNTER_EXHAUSTED', now)
                        return []
                    cid = max(choices, key=lambda c: (self.eligible[c], -c))
                    self.pending = {'event': self.next_event, 'class_id': cid, 'baseline_seq': self.lt['seq'],
                                    'created_at': now, 'last_sent': float('-inf'), 'attempts': 0}
                    self.last_request = dict(self.pending)
                    self.last_request.pop('last_sent')
                    self.latched.add(cid)
                    self.next_event += 1
                    self.total_stop_requests += 1
                    self._event('stop_requested', now, event_id=self.pending['event'], class_id=cid,
                                class_name=NAMES[cid], confidence=self.eligible[cid])
            if self.pending and self.lt['mode'] in (0, 2, 3) and now - self.pending['last_sent'] >= .1 - 1e-9:
                p = self.pending
                commands.append(f'@VC1,S,{self.session},{p["event"]},{p["class_id"]}\n'.encode('ascii'))
                p['last_sent'], p['attempts'] = now, p['attempts'] + 1
            self.last_outgoing = now
            return commands

    def status(self, now):
        with self.lock:
            pending = dict(self.pending) if self.pending else None
            if pending:
                pending.pop('last_sent')
            return {'session': self.session, 'model_ready': self.model_ready, 'serial_connected': self.serial_connected,
                    'serial_fresh': self.lt_at is not None and self.v1_at is not None and
                    now - self.lt_at < self.serial_freshness and now - self.v1_at < self.serial_freshness,
                    'heartbeat_allowed': bool(self._healthy(now)), 'fatal': self.fatal,
                    'mega_mode_id': self.lt['mode'] if self.lt else None,
                    'mega_mode': MODES[self.lt['mode']] if self.lt else 'UNSEEN',
                    'stop_seq': self.lt['seq'] if self.lt else None,
                    'stop_reason': REASONS[self.v1['reason']] if self.v1 else 'UNSEEN',
                    'link_live': bool(self.v1['live']) if self.v1 else False,
                    'pending_event': pending, 'last_ack': dict(self.last_ack) if self.last_ack else None,
                    'last_request': dict(self.last_request) if self.last_request else None,
                    'latched_classes': sorted(self.latched), 'detected_classes': dict(self.visible),
                    'stop_eligible_classes': dict(self.eligible),
                    'total_stop_requests': self.total_stop_requests, 'total_accepted_stops': self.total_accepted_stops,
                    'recent_events': list(self.history)}

    def drain_events(self):
        with self.lock:
            result = list(self.events)
            self.events.clear()
            return result
