"""VC1 link for the MS1 mission firmware: target filtering plus GO/HOLD.

Extends VisionStopController (unchanged S/heartbeat/ack/fault behaviour) with:

* firmware identity: no heartbeat is sent until ``@ID1,MS1,...`` is seen, so
  this program never drives the old 5-second vision_stop firmware;
* a target filter: only the current trial's class may request a stop;
* host commands ``@VC1,G,<session>,<event>`` (start/resume) and
  ``@VC1,P,<session>,<event>`` (hold anywhere), sharing the S event counter,
  retransmitted every 100 ms until acknowledged, 1 s ack timeout -> host fault;
* an unexpected-motion check: leaving STOPPED/WAIT without our GO is a fault.

No hardware access here; the serial thread owns the port.
"""
from __future__ import annotations

from vision_control import REASONS, U32, VisionStopController

MS_REASONS = {**REASONS, 4: 'HOST_HOLD'}
MOVING = (0, 2, 3)
STOPPED, FAULT, WAIT = 1, 4, 5
REQUIRED_FIRMWARE = 'MS1'


class MissionLinkController(VisionStopController):
    REASONS = MS_REASONS

    def __init__(self, session, host_ack_timeout=1.0, **kwargs):
        super().__init__(session, **kwargs)
        self.firmware = None          # (family, build) from @ID1
        self.firmware_at = None
        self.target_class = None      # None: no vision stop may be requested
        self.host_pending = None
        self.host_ack_timeout = host_ack_timeout
        self.last_host_ack = None

    # ---------------- inputs ----------------
    def ingest_line(self, line, now):
        if isinstance(line, bytes) and line.startswith(b'@ID1,'):
            return self._ingest_id(line, now)
        with self.lock:
            old_mode = self.lt['mode'] if self.lt else None
            accepted = super().ingest_line(line, now)
            if accepted and self.lt and old_mode in (STOPPED, WAIT) and self.lt['mode'] in MOVING:
                # A GO is acknowledged by this very LT1 line, so it is still pending here.
                if not (self.host_pending and self.host_pending['kind'] == 'G'):
                    self._fault('UNEXPECTED_RESUME_WITHOUT_GO', now)
            self._check_host_ack(now)
            return accepted

    def _ingest_id(self, line, now):
        parts = line.rstrip(b'\r\n').split(b',')
        if len(parts) != 3 or not all(1 <= len(p) <= 24 for p in parts[1:]):
            return False
        try:
            family, build = parts[1].decode('ascii'), parts[2].decode('ascii')
        except UnicodeDecodeError:
            return False
        with self.lock:
            if self.firmware and self.firmware != (family, build):
                self._fault('FIRMWARE_CHANGED_RESTART_APP', now)
            self.firmware, self.firmware_at = (family, build), now
            if family != REQUIRED_FIRMWARE:
                self._fault(f'WRONG_FIRMWARE_{family}', now)
            return True

    def observe(self, detections, captured_at, finished_at):
        with self.lock:
            super().observe(detections, captured_at, finished_at)
            # Only the current target may be stop-eligible. Everything stays in
            # `visible` for the operator view and logs.
            self.eligible = {cid: c for cid, c in self.eligible.items() if cid == self.target_class}

    def set_target(self, class_id):
        with self.lock:
            if self.target_class != class_id:
                self.target_class = class_id
                self.eligible = {cid: c for cid, c in self.eligible.items() if cid == class_id}
                self._event('target_changed', self.capture_at or 0., class_id=class_id)

    def _healthy(self, now):
        return super()._healthy(now) and self.firmware is not None and self.firmware[0] == REQUIRED_FIRMWARE

    # ---------------- host commands ----------------
    def request_go(self, now):
        """Start from WAIT or resume from STOPPED. Returns the event id or None."""
        return self._request('G', now, (WAIT, STOPPED))

    def request_hold(self, now):
        """Brake wherever the robot is (FOLLOW/EXIT_BAR/SEARCH). Returns id or None."""
        return self._request('P', now, MOVING)

    def _request(self, kind, now, modes):
        with self.lock:
            if (self.fatal or self.pending or self.host_pending or not self._healthy(now)
                    or self.lt['mode'] not in modes):
                return None
            if self.next_event > U32:
                self._fault('EVENT_COUNTER_EXHAUSTED', now)
                return None
            self.host_pending = {'kind': kind, 'event': self.next_event, 'created_at': now,
                                 'last_sent': float('-inf'), 'baseline_seq': self.lt['seq'],
                                 'from_mode': self.lt['mode']}
            self.next_event += 1
            self._event('host_requested', now, command=kind, event_id=self.host_pending['event'])
            return self.host_pending['event']

    def _check_host_ack(self, now):
        p = self.host_pending
        if not p or not self.lt or not self.v1 or self.fatal:
            return
        if now - p['created_at'] >= self.host_ack_timeout:
            self._fault(f'HOST_{p["kind"]}_ACK_TIMEOUT', now)
            return
        v, lt = self.v1, self.lt
        fresh = (self.lt_at >= p['created_at'] and self.v1_at >= p['created_at'] and
                 now - self.lt_at < self.serial_freshness and now - self.v1_at < self.serial_freshness)
        if not fresh or v['session'] != self.session or v['event'] != p['event']:
            return
        if p['kind'] == 'G':
            ok = lt['mode'] in MOVING and v['reason'] == 0
        else:
            ok = lt['mode'] == STOPPED and v['reason'] == 4 and lt['seq'] == ((p['baseline_seq'] + 1) & U32)
        if ok:
            self.last_host_ack = {'kind': p['kind'], 'event': p['event'], 'confirmed_at': now}
            self.host_pending = None
            self._event('host_confirmed', now, command=p['kind'], event_id=p['event'])

    def outgoing(self, now):
        with self.lock:
            if self.host_pending and now - self.host_pending['created_at'] >= self.host_ack_timeout:
                self._fault(f'HOST_{self.host_pending["kind"]}_ACK_TIMEOUT', now)
            # Never start a vision stop while a host command is outstanding.
            hidden = None
            if self.host_pending:
                hidden, self.eligible = self.eligible, {}
            try:
                commands = super().outgoing(now)
            finally:
                if hidden is not None:
                    self.eligible = hidden
            p = self.host_pending
            if p and self._healthy(now) and now - p['last_sent'] >= .1 - 1e-9:
                commands.append(f'@VC1,{p["kind"]},{self.session},{p["event"]}\n'.encode('ascii'))
                p['last_sent'] = now
            return commands

    def status(self, now):
        with self.lock:
            state = super().status(now)
            hp = dict(self.host_pending) if self.host_pending else None
            if hp:
                hp.pop('last_sent')
            state.update(firmware=list(self.firmware) if self.firmware else None,
                         target_class=self.target_class, host_pending=hp,
                         last_host_ack=dict(self.last_host_ack) if self.last_host_ack else None)
            return state
