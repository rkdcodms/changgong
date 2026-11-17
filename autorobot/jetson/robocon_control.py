"""Ordered stops using the class IDs and camera boxes from the statue detector."""
from __future__ import annotations

from vision_control import NAMES, VisionStopController, valid_detection

HORSE_CLASS = 7  # TrojanHorse in the detector mapping
RELEASE_CONFIRM_SECONDS = 1.0
APPROACH_CLEAR_SECONDS = 0.8
# 최종 룰북(Final revised) 27쪽: 랩 완주는 0점. 마지막 목표에서는 석상을 들어 올려도
# 출발 명령(R)을 보내지 않아 그 자리에 계속 정지한다 (Mega는 R을 받기 전까지 제동 유지).
# 이탈·리셋 위험이 없고 오디세우스를 옮기기도 쉽다. False면 예전처럼 계속 주행한다.
HOLD_AFTER_LAST = True


def _area(box):
    return (box[2] - box[0]) * (box[3] - box[1])


def _overlap(previous, current):
    width = max(0., min(previous[2], current[2]) - max(previous[0], current[0]))
    height = max(0., min(previous[3], current[3]) - max(previous[1], current[1]))
    intersection = width * height
    return intersection / (_area(previous) + _area(current) - intersection)


class OrderedStopController(VisionStopController):
    """Hold each stop until its object disappears or a larger rival is detected.

    Entry uses the existing same-object recognition consensus. While stopped,
    the target may move off center, tie another box, or drop below the *entry*
    confidence threshold. All targets require one continuous second of fresh
    frames satisfying their release condition. For TrojanHorse every horse
    must be out of detection. Only the Mega's resume confirmation advances the
    sequence. With hold_after_last the robot never resumes from the last target
    (heartbeats continue, so the Mega stays braked); without it the camera and
    line follower stay active after the last release.
    """
    heartbeat_command = 'Q'

    def __init__(self, session, targets, confidence=.55, min_frames=3, min_span=.1,
                 hold_after_last=HOLD_AFTER_LAST):
        super().__init__(session, confidence=confidence, min_frames=min_frames,
                         min_span=min_span, restart_cooldown=0.)
        self.hold_after_last = bool(hold_after_last)
        self.final_hold = False
        self.targets = tuple(targets)
        if not 1 <= len(self.targets) <= 8 or any(
                type(cid) is not int or cid not in NAMES for cid in self.targets):
            raise ValueError('Expected 1 to 8 target class IDs from 0 to 7')
        self.target_index = 0
        self.approaching = False
        self.approach_missing_since = None
        self.last_approach_command = None
        self.held_box = None
        self.release_since = None
        self.release_ready = False
        self.release_started_at = None
        self.release_last_sent = float('-inf')

    def observe(self, detections, captured_at, finished_at):
        with self.lock:
            boxes = [item for item in detections if valid_detection(item)]
            ranked = sorted(boxes, key=lambda item: _area(item['bbox']), reverse=True)
            largest = ranked[0] if ranked else None
            unique = largest is not None and (
                len(ranked) == 1 or _area(largest['bbox']) > _area(ranked[1]['bbox']))
            target = self.targets[self.target_index] if self.target_index < len(self.targets) else None
            # Keep off-center and low-confidence boxes in the area comparison.
            # Supply the same eligibility flags to the controller and the preview.
            eligible = None
            if (self.held_box is None and unique and largest['class_id'] == target and
                    largest['confidence'] >= self.confidence and largest.get('stop_eligible', True) is True):
                eligible = largest
            reviewed = [dict(item, stop_eligible=item is eligible) for item in boxes]
            previous_capture = self.capture_at
            super().observe(reviewed, captured_at, finished_at)
            if self.capture_at == previous_capture:
                self.approach_missing_since = None
                self.release_since = None
                self.release_ready = False
                return reviewed
            if previous_capture is None or captured_at - previous_capture > self.continuity_gap:
                self.approach_missing_since = None
                self.release_since = None
                self.release_ready = False
            # A first, even off-centre, detection can slow the robot. Only the
            # independent high-confidence consensus below can stop it.
            if target in self.visible or self.held_box is not None:
                self.approaching = True
                self.approach_missing_since = None
            elif self.approaching:
                if self.approach_missing_since is None:
                    self.approach_missing_since = captured_at
                elif captured_at - self.approach_missing_since >= APPROACH_CLEAR_SECONDS - 1e-9:
                    self.approaching = False
                    self.approach_missing_since = None
            if self.held_box is None or self.release_started_at is not None:
                return reviewed
            # Keep the overlapping instance when more than one box has this
            # class. A disjoint same-class replacement is a different object.
            matches = (item for item in boxes if item['class_id'] == target)
            tracked = max(matches, key=lambda item: _overlap(self.held_box, item['bbox']), default=None)
            present = tracked is not None and _overlap(self.held_box, tracked['bbox']) > 0
            if present:
                self.held_box = tuple(tracked['bbox'])
            if self.pending or captured_at < self.last_ack['confirmed_at']:
                return reviewed
            if target == HORSE_CLASS:
                cleared = not any(item['class_id'] == HORSE_CLASS for item in boxes)
            else:
                cleared = not present or _area(self.held_box) < _area(largest['bbox'])
            if not cleared:
                self.release_since = None
                self.release_ready = False
            elif self.release_since is None:
                self.release_since = captured_at
            else:
                self.release_ready = captured_at >= self.release_since + RELEASE_CONFIRM_SECONDS
            return reviewed

    def outgoing(self, now):
        with self.lock:
            if self.release_started_at is not None and now - self.release_started_at >= 1.:
                self._fault('RELEASE_ACK_TIMEOUT', now)
            previous_heartbeat = self.last_heartbeat
            commands = super().outgoing(now)
            if not self._healthy(now):
                return commands
            # Send immediately on a change; repeat with heartbeats so a missed
            # speed command is corrected. A stop command in this batch goes first.
            if (self.v1['session'] == self.session and self.v1['live'] and
                    self.lt['mode'] in (0, 1, 2, 3) and
                    (self.last_approach_command != self.approaching or
                     previous_heartbeat != self.last_heartbeat)):
                commands.append(f'@VC1,A,{self.session},{int(self.approaching)}\n'.encode('ascii'))
                if self.last_approach_command != self.approaching:
                    self._event('approach_speed_requested', now, slow=self.approaching,
                                target_index=self.target_index)
                self.last_approach_command = self.approaching
            if self.held_box is None and self.pending:
                self.held_box = tuple(self.streaks[self.pending['class_id']]['bbox'])
            if self.held_box is None or self.pending:
                return commands
            if self.hold_after_last and self.target_index == len(self.targets) - 1:
                # Last target: never send R. Record once that its statue has been lifted.
                if self.release_ready and not self.final_hold:
                    self.final_hold = True
                    self._event('final_hold', now, event_id=self.last_ack['event'],
                                class_name=self.last_ack['class_name'], total=len(self.targets))
                return commands
            if not self.release_ready and self.release_started_at is None:
                return commands
            ack = self.last_ack
            if (self.lt['mode'] != 1 or self.lt['seq'] != ack['stop_seq'] or
                    self.v1['session'] != self.session or self.v1['event'] != ack['event'] or
                    self.v1['label'] != ack['class_id'] or self.v1['reason'] != 2 or not self.v1['live']):
                return commands
            if self.release_started_at is None:
                self.release_started_at = now
                self._event('release_requested', now, event_id=ack['event'], class_name=ack['class_name'])
            if now - self.release_last_sent >= .1 - 1e-9:
                commands.append(f'@VC1,R,{self.session},{ack["event"]}\n'.encode('ascii'))
                self.release_last_sent = now
            return commands

    def _check_ack(self, now):
        super()._check_ack(now)
        if self.release_started_at is None or self.fatal:
            return
        if now - self.release_started_at >= 1.:
            self._fault('RELEASE_ACK_TIMEOUT', now)
            return
        ack = self.last_ack
        if (self.lt_at < self.release_started_at or self.v1_at < self.release_started_at or
                now - self.lt_at >= self.serial_freshness or now - self.v1_at >= self.serial_freshness or
                self.lt['mode'] not in (0, 2, 3) or self.lt['seq'] != ack['stop_seq'] or
                self.v1['session'] != self.session or self.v1['event'] != ack['event'] or
                self.v1['label'] != ack['class_id'] or self.v1['reason'] != 0 or not self.v1['live']):
            return
        self.target_index += 1
        self.approaching = (self.target_index < len(self.targets) and
                            self.targets[self.target_index] in self.visible)
        self.approach_missing_since = None
        self._event('target_completed', now, event_id=ack['event'], class_name=ack['class_name'],
                    completed=self.target_index, total=len(self.targets))
        self.held_box = None
        self.release_since = None
        self.release_ready = False
        self.release_started_at = None
        self.release_last_sent = float('-inf')
        self.latched.clear()
        self.absent_since.clear()
        self.streaks.clear()
        self.eligible.clear()

    def status(self, now):
        with self.lock:
            state = super().status(now)
            state.update(target_sequence=[NAMES[cid] for cid in self.targets],
                         target_index=self.target_index,
                         approach_requested=self.last_approach_command is True,
                         hold_after_last=self.hold_after_last, final_hold=self.final_hold)
            return state
