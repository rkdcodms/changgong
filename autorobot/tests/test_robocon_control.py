"""OrderedStopController (jetson/robocon_control.py) against a message-level Mega model.

Records the CURRENT behaviour of the Jetson code; no devices are opened.
The Mega model follows arduino/trace_stop/trace_stop.ino for Q/S/R/A.
"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'jetson'))
from robocon_control import OrderedStopController  # noqa: E402

CENTER = (280., 100., 360., 300.)


def box(cid, bbox=CENTER, conf=.9):
    return {'class_id': cid, 'confidence': conf, 'bbox': list(bbox), 'stop_eligible': True}


class Mega:
    def __init__(self):
        self.mode, self.seq, self.session, self.event, self.cls, self.reason = 5, 0, 0, 0, -1, 0
        self.approach = False
        self.ignore_release = False

    def rx(self, line):
        p = line.decode().strip().split(',')
        if p[1] == 'Q' and self.session == 0:
            self.session, self.mode = int(p[2]), 0
        elif p[1] == 'S' and self.mode in (0, 3) and int(p[3]) > self.event:
            self.event, self.cls, self.reason, self.mode = int(p[3]), int(p[4]), 2, 1
            self.seq += 1
        elif p[1] == 'R' and self.mode == 1 and int(p[3]) == self.event and not self.ignore_release:
            self.mode, self.reason = 0, 0
        elif p[1] == 'A':
            self.approach = p[3] == '1'

    def tx(self, t):
        live = 1 if self.session else 0
        return [f'@LT1,{self.mode},{self.seq},0,{int(t * 1000)},0\n'.encode(),
                f'@V1,{self.session},{self.event},{self.cls},{self.reason},{live}\n'.encode()]


class Bench:
    def __init__(self, targets=(5, 2)):
        self.c = OrderedStopController(77, targets)
        self.c.set_model_ready(True, 0)
        self.c.set_serial_connected(True, 0)
        self.m, self.t, self.sent = Mega(), 0., []

    def run(self, seconds, frames):
        for _ in range(int(round(seconds / .05))):
            self.t = round(self.t + .05, 3)
            for line in self.m.tx(self.t):
                self.c.ingest_line(line, self.t)
            self.c.observe(frames(self.t), self.t - .01, self.t)
            for cmd in self.c.outgoing(self.t):
                self.sent.append((self.t, cmd.decode().split(',')[1]))
                self.m.rx(cmd)

    def times(self, kind):
        return [t for t, k in self.sent if k == kind]


class OrderedStopTests(unittest.TestCase):
    def test_statue_still_present_keeps_robot_stopped(self):
        b = Bench(); b.run(12, lambda t: [box(5)])
        self.assertEqual(b.m.mode, 1)
        self.assertEqual(b.times('R'), [])
        self.assertEqual(b.c.target_index, 0)

    def test_release_one_second_after_statue_leaves_and_advance_target(self):
        b = Bench(); b.run(12, lambda t: [box(5)] if t < 6 else [])
        self.assertEqual(b.m.mode, 0)
        self.assertEqual(b.c.target_index, 1)
        self.assertAlmostEqual(b.times('R')[0], 7.0, delta=.1)

    def test_short_gap_below_one_second_does_not_release(self):
        b = Bench(); b.run(12, lambda t: [box(5)] if (t < 6 or t > 6.8) else [])
        self.assertEqual(b.m.mode, 1)
        self.assertEqual(b.times('R'), [])

    def test_known_limitation_occlusion_over_one_second_releases(self):
        # The statue is still on the field but hidden for 1.2 s (e.g. by the manual
        # robot). The code cannot tell this from a lift: it releases. Team plan:
        # drive the manual robot so it never covers the side camera (2026-11-13).
        b = Bench(); b.run(12, lambda t: [box(5)] if (t < 6 or t > 7.2) else [])
        self.assertEqual(b.c.target_index, 1)

    def test_lifted_statue_held_close_to_camera_keeps_stop(self):
        b = Bench(); b.run(12, lambda t: [box(5)] if t < 6 else [box(5, (200., 0., 440., 400.))])
        self.assertEqual(b.m.mode, 1)

    def test_non_target_never_stops_and_approach_follows_target(self):
        b = Bench(); b.run(5, lambda t: [box(6)])
        self.assertEqual(b.times('S'), [])
        self.assertFalse(b.m.approach)
        b.run(.3, lambda t: [box(5, (0., 0., 40., 40.), conf=.4)])   # far, low confidence: slow only
        self.assertTrue(b.m.approach)
        self.assertEqual(b.times('S'), [])

    def test_target_must_be_the_largest_box(self):
        b = Bench(); b.run(5, lambda t: [box(5), box(6, (100., 0., 600., 470.))])
        self.assertEqual(b.times('S'), [])

    def test_release_without_mega_ack_faults(self):
        b = Bench(); b.m.ignore_release = True
        b.run(12, lambda t: [box(5)] if t < 6 else [])
        self.assertEqual(b.c.fatal, 'RELEASE_ACK_TIMEOUT')

    def test_full_sequence_then_keeps_driving(self):
        b = Bench(targets=(5, 2))
        b.run(4, lambda t: [box(5)]); b.run(3, lambda t: [])
        b.run(4, lambda t: [box(2)]); b.run(3, lambda t: [])
        self.assertEqual(b.c.target_index, 2)
        self.assertEqual(len(b.times('S')), 2)
        b.run(4, lambda t: [box(5)])
        self.assertEqual(len(b.times('S')), 2, 'after the last target no further stops')
        self.assertEqual(b.m.mode, 0)

    def test_target_list_validation(self):
        for bad in ((), (8,), tuple(range(8)) + (0,), ('Scylla',)):
            with self.assertRaises(ValueError):
                OrderedStopController(1, bad)


if __name__ == '__main__':
    unittest.main()
