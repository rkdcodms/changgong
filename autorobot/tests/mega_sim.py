"""Message-level model of the MS1 Mega firmware for Jetson-side tests.

It mirrors the protocol rules of arduino/mission_stop/mission_stop.ino that the
C++ harness verifies (session binding, event ordering, hold/GO/P, 1500 ms
heartbeat FAULT). It is not a line-following or motor model.
"""
FOLLOW, STOPPED, EXIT_BAR, SEARCH, FAULT, WAIT = range(6)


class FakeMega:
    def __init__(self, family='MS1', auto_resume=None, start_ms=0):
        self.family = family
        self.auto_resume = auto_resume          # seconds, like vision_stop's 5 s
        self.mode, self.seq, self.reason, self.cls = WAIT, 0, 0, -1
        self.session = self.last_event = 0
        self.last_hb = None
        self.state_since = 0.
        self.boot = start_ms
        self.dropped = False                    # simulate a dead serial line
        self.ignore_go = False

    def _live(self, now):
        return self.session != 0 and self.last_hb is not None and now - self.last_hb < 1.5

    def _mode(self, mode, now):
        if mode == STOPPED and self.mode != STOPPED:
            self.seq += 1
        self.mode, self.state_since = mode, now

    def receive(self, line, now):
        if self.dropped:
            return
        parts = line.decode().rstrip('\n').split(',')
        if parts[0] != '@VC1' or len(parts) < 3:
            return
        session = int(parts[2])
        if parts[1] == 'H' and len(parts) == 3:
            if self.session == 0 and self.mode == WAIT:
                self.session, self.last_hb = session, now
                if self.family != 'MS1':                 # old vision_stop: H starts motion
                    self._mode(FOLLOW, now)
            elif session == self.session:
                self.last_hb = now
            return
        if session != self.session or not self._live(now):
            return
        if parts[1] in ('G', 'P') and len(parts) == 4 and self.family == 'MS1':
            event = int(parts[3])
            if event <= self.last_event:
                return
            if parts[1] == 'G':
                if self.ignore_go:
                    return
                if self.mode == WAIT:
                    self.last_event = event; self._mode(FOLLOW, now)
                elif self.mode == STOPPED and now - self.state_since >= .3 and self.reason != 3:
                    self.last_event, self.reason = event, 0; self._mode(FOLLOW, now)
                return
            if self.mode in (FOLLOW, EXIT_BAR, SEARCH):
                self.last_event, self.reason = event, 4; self._mode(STOPPED, now)
            return
        if parts[1] == 'S' and len(parts) == 5:
            event, cls = int(parts[3]), int(parts[4])
            if event <= self.last_event or self.mode not in (FOLLOW, EXIT_BAR, SEARCH):
                return
            self.last_event, self.cls, self.reason = event, cls, 2
            self._mode(STOPPED, now)

    def step(self, now):
        if self.session and not self._live(now) and self.mode != FAULT:
            self.reason = 3; self._mode(FAULT, now)
        if self.auto_resume and self.mode == STOPPED and now - self.state_since >= self.auto_resume:
            self.reason = 0; self._mode(FOLLOW, now)

    def emit(self, now):
        if self.dropped:
            return []
        uptime = (self.boot + int(now * 1000)) & 0xffffffff
        rows = [f'@LT1,{self.mode},{self.seq},{int((now - self.state_since) * 1000)},{uptime},0\n',
                f'@V1,{self.session},{self.last_event},{self.cls},{self.reason},{1 if self._live(now) else 0}\n']
        if self.family == 'MS1':
            rows.insert(0, '@ID1,MS1,MS1-test\n')
        elif self.family != 'NONE':
            rows.insert(0, f'@ID1,{self.family},x\n')
        return [r.encode() for r in rows]
