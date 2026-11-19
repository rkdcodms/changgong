"""AutoRobot game state: robot motion state separated from per-trial progress.

Pure logic driven by ``tick(now)``; talks to a MissionLinkController-like
``link`` (set_target, request_go, request_hold, status). No hardware access.

Separation required by the 2026-11-12 TA answer (Classum #129): after the
crew of trial n are moved, the AutoRobot may travel to trial n+1 while the
manual robot carries statue n to the altar. Therefore

* motion:  DISARMED, READY, STARTING, SEEKING, STOPPED, DEPARTING,
           FINAL_HOLD, HOST_HOLD, FAULT
* trial n: stop  PENDING -> STOPPED
           crew  PENDING -> DONE | NOT_REQUIRED (crew 0) | UNVERIFIED (timed policy)
           altar PENDING -> DONE
  and "trial complete" means crew satisfied AND altar DONE, independent of
  where the robot is.

How crew_done / departure_clear / altar_done are *sensed* on the contest robot
is not decided yet (see docs). Events therefore carry a ``source``; the
``operator`` source is a development aid and marks the run as not autonomous.
The ``timed`` departure policy leaves after a measured dwell and records the
crew state as UNVERIFIED: it is never reported as completed.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import time

MOVING = ('FOLLOW', 'EXIT_BAR', 'SEARCH')
EVENT_KINDS = ('start', 'crew_done', 'departure_clear', 'altar_done', 'hold', 'resume')


@dataclass
class MissionConfig:
    departure_policy: str = 'events'      # 'events' | 'timed'
    min_dwell_s: float = 1.0               # never leave a statue sooner than this
    timed_base_s: float = 8.0              # timed policy: base + per_crew * crew
    timed_per_crew_s: float = 6.0
    after_last_trial: str = 'hold'         # 'hold' (stay for Odysseus) | 'continue'
    continue_seconds: float = 0.0          # 'continue': drive this long, then hold
    game_seconds: float = 300.0            # 5 min (rulebook p.29); keep configurable
    seek_warn_s: float = 45.0              # log once if a target takes this long
    max_event_age_s: float = 5.0           # reject delayed events
    link_stale_s: float = 1.0              # no heartbeat this long -> mission FAULT
    start_index: int = 0                   # recovery after a reset: resume at trial k
    allowed_sources: tuple = ('operator', 'sensor')

    def validate(self):
        if self.departure_policy not in ('events', 'timed'):
            raise ValueError('departure_policy must be events or timed')
        if self.after_last_trial not in ('hold', 'continue'):
            raise ValueError('after_last_trial must be hold or continue')
        for name in ('min_dwell_s', 'timed_base_s', 'timed_per_crew_s', 'continue_seconds',
                     'game_seconds', 'seek_warn_s', 'max_event_age_s', 'link_stale_s'):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or value < 0:
                raise ValueError(f'{name} must be a non-negative number')
        if self.min_dwell_s < 0.5:
            raise ValueError('min_dwell_s must be at least 0.5 s (firmware MIN_STOP_MS is 0.3 s)')
        return self


@dataclass
class TrialState:
    n: int
    name: str
    class_id: int
    crew_required: int
    stop: str = 'PENDING'
    crew: str = 'PENDING'
    altar: str = 'PENDING'
    seek_started_at: float = None
    stopped_at: float = None
    crew_done_at: float = None
    departure_clear_at: float = None
    departed_at: float = None
    altar_done_at: float = None
    stop_event: int = None
    previous_altar_pending_on_arrival: bool = False
    sources: list = field(default_factory=list)

    @property
    def crew_satisfied(self):
        return self.crew in ('DONE', 'NOT_REQUIRED', 'UNVERIFIED')

    @property
    def complete(self):
        return self.crew in ('DONE', 'NOT_REQUIRED') and self.altar == 'DONE'


class MissionRunner:
    def __init__(self, plan, link, config=None, clock=time.monotonic, wall=time.time):
        self.plan, self.link = plan, link
        self.config = (config or MissionConfig()).validate()
        if not 0 <= self.config.start_index < len(plan.trials):
            raise ValueError('start_index outside the plan')
        self.clock, self.wall = clock, wall
        self.trials = [TrialState(t.n, t.name, t.class_id, t.crew) for t in plan.trials]
        for t in self.trials[:self.config.start_index]:
            t.stop = 'SKIPPED_BEFORE_RESTART'
        self.motion = 'DISARMED'
        self.current = self.config.start_index
        self.started_at = None
        self.fault = None
        self.seen_ids = set()
        self.events = []
        self.log = []
        self.operator_used = False
        self.pending_go = self.pending_hold = None
        self.hold_return = None
        self.continue_until = None
        self.last_stop_event = None
        self.seek_warned = False
        self._non_target_seen = set()
        self._unhealthy_since = None

    # ---------------- helpers ----------------
    def _emit(self, name, now, **fields):
        row = {'event': name, 'at': round(now, 3), 'motion': self.motion,
               'game_s': None if self.started_at is None else round(now - self.started_at, 3), **fields}
        self.events.append(row)
        self.log.append(row)
        return row

    def _set_motion(self, motion, now, **fields):
        if motion != self.motion:
            old, self.motion = self.motion, motion
            self._emit('motion', now, previous=old, **fields)

    def _trial(self, n):
        if type(n) is not int or not 1 <= n <= len(self.trials):
            return None
        return self.trials[n - 1]

    def _fail(self, reason, now):
        if self.fault is None:
            self.fault = reason
            self.link.set_target(None)
            self._set_motion('FAULT', now, reason=reason)

    def drain(self):
        out, self.events = self.events, []
        return out

    # ---------------- external events ----------------
    def submit(self, event, now=None):
        """Validate one event dict. Returns (accepted: bool, reason: str)."""
        now = self.clock() if now is None else now
        kind, n, eid = event.get('kind'), event.get('trial'), event.get('id')
        source = event.get('source', 'operator')
        result = self._validate(kind, n, eid, source, event.get('created_unix'))
        if result is None:
            result = self._apply(kind, n, source, now)
        accepted, reason = result
        if isinstance(eid, str) and eid and accepted:
            self.seen_ids.add(eid)
        if accepted and source == 'operator':
            self.operator_used = True
        self._emit('input', now, kind=kind, trial=n, id=eid, source=source, accepted=accepted, reason=reason)
        return accepted, reason

    def _validate(self, kind, n, eid, source, created):
        if kind not in EVENT_KINDS:
            return False, 'unknown_kind'
        if not isinstance(eid, str) or not 1 <= len(eid) <= 64:
            return False, 'missing_id'
        if eid in self.seen_ids:
            return False, 'duplicate_id'
        if source not in self.config.allowed_sources:
            return False, 'source_not_allowed'
        if created is not None:
            if not isinstance(created, (int, float)) or isinstance(created, bool):
                return False, 'bad_timestamp'
            age = self.wall() - created
            if age > self.config.max_event_age_s or age < -1.0:
                return False, 'stale_or_future_event'
        if self.fault:
            return False, 'faulted'
        return None

    def _apply(self, kind, n, source, now):
        if kind == 'start':
            if self.motion != 'READY':
                return False, f'not_ready:{self.motion}'
            event = self.link.request_go(now)
            if event is None:
                return False, 'link_refused'
            self.pending_go = event
            self.started_at = now
            self._set_motion('STARTING', now)
            return True, 'go_sent'
        if kind == 'hold':
            if self.motion not in ('SEEKING', 'DEPARTING') and not (
                    self.motion == 'FINAL_HOLD' and self.continue_until is not None):
                return False, f'cannot_hold:{self.motion}'
            event = self.link.request_hold(now)
            if event is None:
                return False, 'link_refused'
            self.pending_hold, self.hold_return = event, self.motion
            return True, 'hold_sent'
        if kind == 'resume':
            if self.motion != 'HOST_HOLD':
                return False, f'not_held:{self.motion}'
            event = self.link.request_go(now)
            if event is None:
                return False, 'link_refused'
            self.pending_go = event
            self._set_motion('DEPARTING', now, resume=True)
            return True, 'go_sent'
        trial = self._trial(n)
        if trial is None:
            return False, 'bad_trial'
        if kind == 'crew_done':
            if trial.crew in ('DONE', 'NOT_REQUIRED'):
                return False, 'already_done'
            if trial.stop != 'STOPPED' or self.current != trial.n - 1 or self.motion != 'STOPPED':
                return False, 'robot_not_stopped_at_this_trial'
            trial.crew, trial.crew_done_at = 'DONE', now
            trial.sources.append(('crew_done', source))
            self._emit('crew_done', now, trial=trial.n, wait_s=round(now - trial.stopped_at, 3))
            return True, 'ok'
        if kind == 'departure_clear':
            if self.motion != 'STOPPED' or self.current != trial.n - 1:
                return False, 'robot_not_stopped_at_this_trial'
            if not trial.crew_satisfied:
                return False, 'crew_pending'
            if trial.departure_clear_at is not None:
                return False, 'already_clear'
            trial.departure_clear_at = now
            trial.sources.append(('departure_clear', source))
            return True, 'ok'
        if kind == 'altar_done':
            if trial.altar == 'DONE':
                return False, 'already_done'
            if not trial.crew_satisfied:
                # Rulebook p.30 (Trial Rule): statue only after the required crew.
                return False, 'crew_not_done_for_this_trial'
            trial.altar, trial.altar_done_at = 'DONE', now
            trial.sources.append(('altar_done', source))
            self._emit('altar_done', now, trial=trial.n, robot_at=self.current + 1, robot_motion=self.motion,
                       complete=trial.complete)
            return True, 'ok'
        return False, 'unhandled'

    # ---------------- periodic update ----------------
    def tick(self, now=None):
        now = self.clock() if now is None else now
        status = self.link.status(now)
        if self.fault:
            return
        if status.get('fatal'):
            return self._fail('LINK:' + str(status['fatal']), now)
        mode = status.get('mega_mode')
        if mode == 'FAULT':
            return self._fail('MEGA_FAULT_' + str(status.get('stop_reason')), now)
        active = self.motion not in ('DISARMED', 'READY')
        if active and not status.get('heartbeat_allowed'):
            if self._unhealthy_since is None:
                self._unhealthy_since = now
            elif now - self._unhealthy_since >= self.config.link_stale_s:
                # No heartbeat is being sent; the Mega watchdog FAULTs at 1.5 s.
                return self._fail('LINK_OR_CAMERA_STALE', now)
        else:
            self._unhealthy_since = None
        if self.started_at is not None and now - self.started_at >= self.config.game_seconds and \
                not getattr(self, '_time_warned', False):
            self._time_warned = True
            self._emit('game_time_elapsed', now)

        if self.motion == 'DISARMED':
            if status.get('heartbeat_allowed') and status.get('link_live') and mode == 'WAIT_VISION':
                self._set_motion('READY', now)
            return
        if self.motion == 'READY':
            if mode != 'WAIT_VISION':
                return self._fail(f'UNEXPECTED_MODE_IN_READY_{mode}', now)
            return
        ack = status.get('last_host_ack') or {}
        if self.pending_go is not None:
            if ack.get('kind') == 'G' and ack.get('event') == self.pending_go:
                self.pending_go = None
                self._after_go(now)
            return
        if self.pending_hold is not None:
            if ack.get('kind') == 'P' and ack.get('event') == self.pending_hold:
                self.pending_hold = None
                if self.hold_return == 'FINAL_HOLD' and self.continue_until is None:
                    self.hold_return = None
                    self._set_motion('FINAL_HOLD', now, parked=True)
                    self._emit('final_hold', now, note='parked after continue_seconds')
                else:
                    self._set_motion('HOST_HOLD', now, held_from=self.hold_return)
            return

        if self.motion == 'SEEKING':
            if mode == 'STOPPED':
                stop = status.get('last_ack') or {}
                trial = self.trials[self.current]
                if stop.get('event') != self.last_stop_event and stop.get('class_id') == trial.class_id:
                    self.last_stop_event = stop.get('event')
                    self._arrive(trial, stop, now)
                elif status.get('pending_event'):
                    pass        # S sent; LT1 STOPPED can precede the V1 ack line
                else:
                    return self._fail('STOPPED_WITHOUT_CONFIRMED_TARGET', now)
            elif not self.seek_warned and now - self.trials[self.current].seek_started_at >= self.config.seek_warn_s:
                self.seek_warned = True
                self._emit('seek_slow', now, trial=self.current + 1, seconds=self.config.seek_warn_s)
            for cid in (status.get('detected_classes') or {}):
                if cid != self.trials[self.current].class_id and cid not in self._non_target_seen:
                    self._non_target_seen.add(cid)
                    self._emit('non_target_seen', now, class_id=cid, target=self.trials[self.current].class_id)
            return
        if self.motion == 'STOPPED':
            if mode != 'STOPPED':
                return self._fail(f'LEFT_STOP_WITHOUT_GO_{mode}', now)
            trial = self.trials[self.current]
            if self._may_depart(trial, now):
                self._depart(trial, now)
            return
        if self.motion == 'FINAL_HOLD' and self.continue_until is not None and now >= self.continue_until:
            event = self.link.request_hold(now)
            if event is not None:
                self.continue_until = None
                self.pending_hold, self.hold_return = event, 'FINAL_HOLD'

    def _after_go(self, now):
        if self.motion == 'STARTING' or (self.motion == 'DEPARTING' and self.hold_return == 'SEEKING'):
            self.hold_return = None
            return self._seek(self.current, now)
        if self.motion == 'DEPARTING':
            if self.hold_return == 'FINAL_HOLD' or self.current >= len(self.trials):
                self.hold_return = None
                return self._set_motion('FINAL_HOLD', now, continuing=True)
            self.hold_return = None
            return self._seek(self.current, now)

    def _seek(self, index, now):
        trial = self.trials[index]
        trial.seek_started_at = now
        self.seek_warned = False
        self._non_target_seen = set()
        self.link.set_target(trial.class_id)
        self._set_motion('SEEKING', now, trial=trial.n, target=trial.name)

    def _arrive(self, trial, stop, now):
        trial.stop, trial.stopped_at, trial.stop_event = 'STOPPED', now, stop.get('event')
        self.link.set_target(None)
        if trial.crew_required == 0:
            trial.crew = 'NOT_REQUIRED'
        if trial.n > 1:
            previous = self.trials[trial.n - 2]
            trial.previous_altar_pending_on_arrival = previous.altar != 'DONE'
        self._set_motion('STOPPED', now, trial=trial.n)
        self._emit('stop_confirmed', now, trial=trial.n, statue=trial.name,
                   seek_s=round(now - trial.seek_started_at, 3),
                   previous_altar_pending=trial.previous_altar_pending_on_arrival)

    def _dwell_needed(self, trial):
        if self.config.departure_policy == 'timed':
            return max(self.config.min_dwell_s,
                       self.config.timed_base_s + self.config.timed_per_crew_s * trial.crew_required)
        return self.config.min_dwell_s

    def _may_depart(self, trial, now):
        if now - trial.stopped_at < self._dwell_needed(trial):
            return False
        if self.config.departure_policy == 'timed':
            if trial.crew == 'PENDING':
                trial.crew = 'UNVERIFIED'
                self._emit('crew_unverified_timed_departure', now, trial=trial.n)
            return True
        return trial.crew_satisfied and trial.departure_clear_at is not None

    def _depart(self, trial, now):
        last = trial.n == len(self.trials)
        if last and self.config.after_last_trial == 'hold':
            if trial.departed_at is None:
                trial.departed_at = now
                self._set_motion('FINAL_HOLD', now, trial=trial.n)
                self._emit('final_hold', now, note='robot stays for the Odysseus transfer')
            return
        event = self.link.request_go(now)
        if event is None:
            return              # retried next tick; link faults surface via status
        trial.departed_at = now
        self.pending_go = event
        self.current += 1
        if last:
            self.continue_until = now + self.config.continue_seconds
            self.hold_return = 'FINAL_HOLD'
        self._set_motion('DEPARTING', now, trial=trial.n, dwell_s=round(now - trial.stopped_at, 3))

    # ---------------- reporting ----------------
    def status(self, now=None):
        now = self.clock() if now is None else now
        current = self.trials[self.current] if self.current < len(self.trials) else None
        return {
            'motion': self.motion, 'fault': self.fault,
            'current_trial': current.n if current else None,
            'target': current.name if current and self.motion in ('SEEKING', 'DEPARTING', 'STARTING') else None,
            'game_elapsed_s': None if self.started_at is None else round(now - self.started_at, 3),
            'game_seconds': self.config.game_seconds,
            'departure_policy': self.config.departure_policy,
            # True only if no operator (development) input was used in this run.
            'autonomous_run': not self.operator_used,
            'operator_input_used': self.operator_used,
            'trials': [{'n': t.n, 'statue': t.name, 'crew_required': t.crew_required, 'stop': t.stop,
                        'crew': t.crew, 'altar': t.altar, 'complete': t.complete,
                        'previous_altar_pending_on_arrival': t.previous_altar_pending_on_arrival}
                       for t in self.trials],
        }
