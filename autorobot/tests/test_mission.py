"""Mission plan, MS1 link and trial state machine. Pure simulation; no devices."""
from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'jetson'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from mission_plan import build_plan, canonical, load_plan, parse_cli
from mission_link import MissionLinkController
from mission import MissionConfig, MissionRunner
import mega_sim

CENTER = [280., 100., 360., 300.]   # centre 0.5 of a 640-wide frame


def det(cid, conf=.9, box=CENTER):
    return {'class_id': cid, 'confidence': conf, 'bbox': list(box), 'stop_eligible': True}


class PlanTests(unittest.TestCase):
    def test_aliases_map_to_model_names(self):
        self.assertEqual(canonical("Poseidon's Wrath"), 'Poseidon')
        self.assertEqual(canonical('Cyclops'), 'Cyclopes')
        self.assertEqual(canonical('Cattle of Helios'), 'Cattle')
        self.assertEqual(canonical('Sirens'), 'Siren')
        self.assertEqual(canonical('trojan horse'), 'TrojanHorse')
        with self.assertRaises(ValueError):
            canonical('Medusa')

    def test_counts_and_ranges(self):
        plan = parse_cli('Scylla, Cyclops, Siren', '2,0,3')
        self.assertEqual([(t.n, t.name, t.crew) for t in plan.trials],
                         [(1, 'Scylla', 2), (2, 'Cyclopes', 0), (3, 'Siren', 3)])
        for order, crews in (('Scylla,Siren', '1,1'), (','.join(['Scylla'] * 9), ','.join(['1'] * 9)),
                             ('Scylla,Siren,Calypso', '1,4,0'), ('Scylla,Siren,Calypso', '1,-1,0'),
                             ('Scylla,Siren,Calypso', '1,1'), ('Scylla,Siren,Scylla', '1,1,1'),
                             ('Scylla,Siren,Calypso', '1,x,1')):
            with self.assertRaises(ValueError, msg=(order, crews)):
                parse_cli(order, crews)
        self.assertEqual(len(build_plan(['Scylla', 'Siren', 'Scylla'], [1, 1, 1], allow_repeat=True).trials), 3)
        with self.assertRaises(ValueError):
            build_plan(['Scylla', 'Siren', 'Calypso'], [1, True, 1])

    def test_all_eight_and_file_round_trip(self):
        plan = parse_cli('Calypso,Cattle,Cyclopes,Laestrygonians,Poseidon,Scylla,Siren,TrojanHorse', '0 1 2 3 0 1 2 3')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'plan.json'
            path.write_text(json.dumps(plan.to_json()))
            self.assertEqual(load_plan(path), plan)
            bad = plan.to_json(); bad['trials'][0]['class_id'] = 5
            path.write_text(json.dumps(bad))
            with self.assertRaises(ValueError):
                load_plan(path)


class Bench:
    """Steps a MissionLinkController against FakeMega in simulated time."""
    def __init__(self, plan=None, config=None, family='MS1', dt=.02):
        self.t, self.dt = 0., dt
        self.mega = mega_sim.FakeMega(family=family)
        # Same controller settings as jetson/mission_server.py.
        self.link = MissionLinkController(4242, min_span=.1, min_frames=3, frame_freshness=.5)
        self.link.set_model_ready(True, 0); self.link.set_serial_connected(True, 0)
        self.plan = plan or parse_cli('Scylla,Cyclops,Siren', '2,0,1')
        self.runner = MissionRunner(self.plan, self.link, config or MissionConfig(), clock=lambda: self.t,
                                    wall=lambda: 1_000_000 + self.t)
        self.visible = []
        self.sent = []
        self.camera_ok = True

    def step(self, n=1):
        for _ in range(n):
            self.t = round(self.t + self.dt, 6)
            self.mega.step(self.t)
            for line in self.mega.emit(self.t):
                self.link.ingest_line(line, self.t)
            if self.camera_ok:
                self.link.observe(list(self.visible), self.t - .01, self.t)
            for cmd in self.link.outgoing(self.t):
                self.sent.append(cmd)
                self.mega.receive(cmd, self.t)
            self.runner.tick(self.t)

    def until(self, predicate, limit=20.):
        end = self.t + limit
        while not predicate():
            if self.t > end:
                raise AssertionError(f'timeout at t={self.t}: motion={self.runner.motion} '
                                     f'fault={self.runner.fault} mega={self.mega.mode}')
            self.step()

    def event(self, kind, trial=None, eid=None, **extra):
        self._n = getattr(self, '_n', 0) + 1
        e = {'kind': kind, 'trial': trial, 'id': eid or f'e{self._n}', 'created_unix': 1_000_000 + self.t, **extra}
        return self.runner.submit(e, self.t)

    def start(self):
        self.until(lambda: self.runner.motion == 'READY')
        self.assertMega(mega_sim.WAIT)
        ok, why = self.event('start')
        assert ok, why
        self.until(lambda: self.runner.motion == 'SEEKING')

    def arrive(self, cid):
        self.visible = [det(cid)]
        self.until(lambda: self.runner.motion == 'STOPPED')
        self.visible = []

    def assertMega(self, mode):
        assert self.mega.mode == mode, (self.mega.mode, mode)


class LinkTests(unittest.TestCase):
    def test_no_heartbeat_without_ms1_identity(self):
        for family in ('NONE', 'VS1'):
            b = Bench(family=family)
            b.step(100)
            self.assertFalse(any(c.startswith(b'@VC1,H') for c in b.sent), family)
            self.assertEqual(b.mega.mode, mega_sim.WAIT, 'old firmware must never be armed')
            if family == 'VS1':
                self.assertIn('WRONG_FIRMWARE', b.link.fatal)

    def test_firmware_change_mid_run_faults(self):
        b = Bench(); b.step(30)
        b.link.ingest_line(b'@ID1,MS1,other-build\n', b.t)
        self.assertEqual(b.link.fatal, 'FIRMWARE_CHANGED_RESTART_APP')

    def test_non_target_stable_statue_never_requests_stop(self):
        b = Bench(); b.start()
        b.visible = [det(6), det(0, box=[0, 0, 60, 60])]   # Siren centred, not the target
        b.step(150)
        self.assertEqual(b.mega.mode, mega_sim.FOLLOW)
        self.assertFalse(any(c.startswith(b'@VC1,S') for c in b.sent))
        self.assertTrue(any(e['event'] == 'non_target_seen' for e in b.runner.log))

    def test_go_ack_timeout_faults_and_no_more_commands(self):
        b = Bench(); b.until(lambda: b.runner.motion == 'READY')
        b.mega.ignore_go = True
        b.event('start')
        b.until(lambda: b.runner.motion == 'FAULT')
        self.assertIn('HOST_G_ACK_TIMEOUT', b.runner.fault)
        n = len(b.sent); b.step(10)
        self.assertEqual(len(b.sent), n, 'faulted link sends nothing, so the Mega watchdog stops it')
        b.step(100)
        self.assertEqual(b.mega.mode, mega_sim.FAULT)

    def test_unexpected_resume_without_go_faults(self):
        b = Bench(); b.start(); b.arrive(5); b.step(50)
        b.mega.mode = mega_sim.FOLLOW            # e.g. old 5 s firmware behaviour
        b.step(3)
        self.assertEqual(b.runner.motion, 'FAULT')
        self.assertIn('UNEXPECTED_RESUME_WITHOUT_GO', b.runner.fault)

    def test_mega_reset_detected(self):
        b = Bench(); b.start()
        b.mega.boot -= 100_000_000               # uptime jumps backwards
        b.step(3)
        self.assertEqual(b.runner.motion, 'FAULT')

    def test_serial_silence_lets_mega_watchdog_fault(self):
        b = Bench(); b.start()
        b.mega.dropped = True
        b.step(100)
        self.assertEqual(b.runner.motion, 'FAULT')
        self.assertFalse(b.link.status(b.t)['heartbeat_allowed'])

    def test_camera_failure_stops_heartbeat_and_faults(self):
        b = Bench(); b.start()
        b.camera_ok = False                        # frames stop arriving
        lost = b.t
        b.until(lambda: b.mega.mode == mega_sim.FAULT, limit=3)
        self.assertLessEqual(b.t - lost, 2.05, 'camera loss -> Mega FAULT within 0.5 s + 1.5 s')
        self.assertEqual(b.runner.motion, 'FAULT')
        b2 = Bench(); b2.start()
        b2.link.set_model_ready(False, b2.t)       # detector thread reported an error
        b2.step(2)
        self.assertEqual(b2.runner.motion, 'FAULT')


class MissionTests(unittest.TestCase):
    def test_full_game_with_lead_move_and_separate_altar_state(self):
        b = Bench(); b.start()
        self.assertEqual(b.link.target_class, 5)                         # Scylla
        b.arrive(5)
        r = b.runner
        self.assertEqual((r.trials[0].stop, r.trials[0].crew, r.trials[0].altar), ('STOPPED', 'PENDING', 'PENDING'))
        b.step(200)                                                       # 4 s: never auto-restarts
        self.assertEqual(b.mega.mode, mega_sim.STOPPED)
        self.assertEqual(b.event('departure_clear', 1), (False, 'crew_pending'))
        self.assertEqual(b.event('altar_done', 1), (False, 'crew_not_done_for_this_trial'))
        self.assertTrue(b.event('crew_done', 1)[0])
        b.step(50); self.assertEqual(b.mega.mode, mega_sim.STOPPED, 'crew done alone does not depart')
        self.assertTrue(b.event('departure_clear', 1)[0])
        b.until(lambda: r.motion == 'SEEKING' and r.current == 1)
        self.assertEqual(b.link.target_class, 2)                         # Cyclopes
        self.assertEqual(r.trials[0].altar, 'PENDING', 'moving on is not trial completion')
        self.assertFalse(r.trials[0].complete)
        self.assertTrue(b.event('altar_done', 1)[0])                     # while robot seeks trial 2
        self.assertTrue(r.trials[0].complete)
        b.arrive(2)
        self.assertEqual(r.trials[1].crew, 'NOT_REQUIRED')
        self.assertTrue(b.event('departure_clear', 2)[0])
        b.until(lambda: r.motion == 'SEEKING' and r.current == 2)
        b.arrive(6)
        self.assertTrue(r.trials[2].previous_altar_pending_on_arrival)
        self.assertTrue(b.event('crew_done', 3)[0]); self.assertTrue(b.event('departure_clear', 3)[0])
        b.until(lambda: r.motion == 'FINAL_HOLD')
        b.step(200)
        self.assertEqual(b.mega.mode, mega_sim.STOPPED, 'robot stays for the Odysseus transfer')
        self.assertTrue(b.event('altar_done', 2)[0]); self.assertTrue(b.event('altar_done', 3)[0])
        self.assertEqual([t['complete'] for t in r.status()['trials']], [True, True, True])
        self.assertFalse(r.status()['autonomous_run'])
        kinds = [e['event'] for e in r.log]
        self.assertEqual(kinds.count('stop_confirmed'), 3)

    def test_duplicate_delayed_and_wrong_trial_events(self):
        b = Bench(); b.start()
        self.assertEqual(b.event('crew_done', 1), (False, 'robot_not_stopped_at_this_trial'))
        b.arrive(5)
        self.assertEqual(b.event('crew_done', 2), (False, 'robot_not_stopped_at_this_trial'))
        self.assertEqual(b.event('crew_done', 9), (False, 'bad_trial'))
        self.assertTrue(b.event('crew_done', 1, eid='X')[0])
        self.assertEqual(b.event('crew_done', 1, eid='X'), (False, 'duplicate_id'))
        self.assertEqual(b.event('crew_done', 1), (False, 'already_done'))
        self.assertEqual(b.event('departure_clear', 1, created_unix=1_000_000 + b.t - 30),
                         (False, 'stale_or_future_event'))
        self.assertEqual(b.event('departure_clear', 1, source='radio'), (False, 'source_not_allowed'))
        self.assertEqual(b.event('start'), (False, 'not_ready:STOPPED'))
        self.assertEqual(b.event('teleport', 1), (False, 'unknown_kind'))
        self.assertTrue(b.event('departure_clear', 1)[0])
        b.until(lambda: b.runner.motion == 'SEEKING')
        # A late crew_done for the finished trial must not count for trial 2.
        self.assertEqual(b.event('crew_done', 1), (False, 'already_done'))
        b.arrive(2)
        self.assertEqual(b.event('crew_done', 2), (False, 'already_done'))   # crew 0 trial

    def test_restart_does_not_redo_previous_trial_after_reconnect(self):
        b = Bench(); b.start(); b.arrive(5)
        b.event('crew_done', 1); b.event('departure_clear', 1)
        b.until(lambda: b.runner.current == 1 and b.runner.motion == 'SEEKING')
        b.visible = [det(5)]                    # Scylla seen again later on the lap
        b.step(150)
        self.assertEqual(b.mega.mode, mega_sim.FOLLOW, 'previous target never re-stops the robot')

    def test_timed_policy_departs_but_never_claims_crew_done(self):
        cfg = MissionConfig(departure_policy='timed', timed_base_s=2, timed_per_crew_s=1)
        b = Bench(config=cfg); b.start(); b.arrive(5)
        stopped = b.t
        b.until(lambda: b.runner.motion == 'SEEKING')
        self.assertGreaterEqual(b.t - stopped, 4.0)                       # 2 + 1 * 2 crew
        self.assertEqual(b.runner.trials[0].crew, 'UNVERIFIED')
        self.assertTrue(b.event('altar_done', 1)[0])
        self.assertFalse(b.runner.trials[0].complete)
        self.assertTrue(b.runner.status()['autonomous_run'] is False)       # operator altar input used

    def test_host_hold_and_resume_while_seeking(self):
        b = Bench(); b.start()
        self.assertTrue(b.event('hold')[0])
        b.until(lambda: b.runner.motion == 'HOST_HOLD')
        self.assertEqual(b.mega.reason, 4)
        b.visible = [det(5)]; b.step(50)
        self.assertEqual(b.mega.mode, mega_sim.STOPPED, 'no vision stop while held')
        b.visible = []
        self.assertTrue(b.event('resume')[0])
        b.until(lambda: b.runner.motion == 'SEEKING')
        b.arrive(5)
        self.assertEqual(b.runner.trials[0].stop, 'STOPPED')

    def test_recovery_start_index(self):
        b = Bench(config=MissionConfig(start_index=1)); b.start()
        self.assertEqual(b.link.target_class, 2)
        self.assertEqual(b.runner.trials[0].stop, 'SKIPPED_BEFORE_RESTART')

    def test_continue_after_last_then_park(self):
        plan = parse_cli('Scylla,Cyclops,Siren', '0,0,0')
        b = Bench(plan=plan, config=MissionConfig(after_last_trial='continue', continue_seconds=2))
        b.start()
        for n, cid in ((1, 5), (2, 2), (3, 6)):
            b.arrive(cid); b.event('departure_clear', n)
            if n < 3:
                b.until(lambda: b.runner.motion == 'SEEKING')
        b.until(lambda: b.runner.motion == 'FINAL_HOLD' and b.mega.mode == mega_sim.FOLLOW)
        b.until(lambda: b.mega.mode == mega_sim.STOPPED, limit=5)
        b.step(5)
        self.assertEqual(b.runner.motion, 'FINAL_HOLD')

    def test_config_validation(self):
        for bad in (dict(departure_policy='x'), dict(min_dwell_s=.1), dict(game_seconds=-1),
                    dict(after_last_trial='laps')):
            with self.assertRaises(ValueError):
                MissionConfig(**bad).validate()
        with self.assertRaises(ValueError):
            MissionRunner(parse_cli('Scylla,Cyclops,Siren', '0,0,0'), None, MissionConfig(start_index=3))


if __name__ == '__main__':
    unittest.main()
