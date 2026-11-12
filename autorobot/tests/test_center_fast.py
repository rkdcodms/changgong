"""Pure tests: no device, network connection, camera, model load or motor access."""
from pathlib import Path
import io
import json
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

# Import only the portable package under test.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'jetson'))
from vision_control import BoundedLines, VisionStopController, parse_line, U32
from vision_stop_server import VisionHub, rotate_input, stop_eligible, parse_sensor_line, drain_startup_serial
from web_preview import FrameHub, PreviewHandler, multipart_frame, PAGE, client_is_allowed


def detection(cid=0, confidence=.8):
    return {'class_id': cid, 'confidence': confidence, 'bbox': [10., 20., 100., 200.]}


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.core = VisionStopController(77)
        self.core.set_model_ready(True, 0)
        self.core.set_serial_connected(True, 0)

    def telemetry(self, t, mode=0, seq=9, session=77, event=0, cid=-1, reason=0, live=1):
        self.core.ingest_line(f'@LT1,{mode},{seq},0,{int(t*1000)},0\n'.encode(), t)
        self.core.ingest_line(f'@V1,{session},{event},{cid},{reason},{live}\n'.encode(), t)

    def frame(self, t, items=None, **telemetry):
        self.telemetry(t, **telemetry)
        self.core.observe([detection()] if items is None else items, t, t+.01)

    def request(self, mode=0, seq=9):
        for t in (0, .1, .2):
            self.frame(t, mode=mode, seq=seq)
        return self.core.outgoing(.22)

    def acknowledge(self, t=.3, mode_after=None, seq=10):
        self.telemetry(t, mode=1, seq=seq, event=1, cid=0, reason=2)
        if mode_after is not None:
            self.telemetry(t+.1, mode=mode_after, seq=seq, event=1, cid=0, reason=2)

    def test_boot_session_zero_can_send_first_heartbeat_but_no_stop(self):
        self.frame(0, mode=5, session=0, live=0)
        self.assertEqual(self.core.outgoing(.01), [b'@VC1,H,77\n'])
        self.assertEqual(self.core.total_stop_requests, 0)

    def test_missing_new_protocol_or_old_inference_prevents_heartbeat(self):
        self.core.ingest_line(b'@LT1,0,9,0,0,0\n', 0)
        self.core.observe([detection()], 0, .01)
        self.assertEqual(self.core.outgoing(.02), [])
        self.telemetry(2)
        self.assertEqual(self.core.outgoing(2.01), [])

    def test_stale_either_serial_stream_prevents_heartbeat(self):
        self.frame(0)
        self.core.observe([], .8, .81)
        self.core.ingest_line(b'@LT1,0,9,0,800,0\n', .8)
        self.assertEqual(self.core.outgoing(.81), [])

    def test_three_frames_and_elapsed_span_both_required(self):
        for t in (0, .01, .02):
            self.frame(t)
        self.assertFalse(any(b',S,' in x for x in self.core.outgoing(.03)))
        self.frame(.2)
        self.assertIn(b'@VC1,S,77,1,0\n', self.core.outgoing(.22))

    def test_all_moving_modes_accept_stable_detection(self):
        for mode in (0, 2, 3):
            with self.subTest(mode=mode):
                self.setUp()
                self.assertIn(b'@VC1,S,77,1,0\n', self.request(mode))

    def test_wait_stopped_fault_do_not_generate_stop(self):
        for mode in (1, 4, 5):
            with self.subTest(mode=mode):
                self.setUp()
                self.assertFalse(any(b',S,' in x for x in self.request(mode)))

    def test_retry_same_event_and_timeout_halts_heartbeat(self):
        self.request()
        self.assertEqual(self.core.outgoing(.25), [])
        self.assertEqual(self.core.outgoing(.32), [b'@VC1,S,77,1,0\n'])
        self.frame(1.21)
        self.assertEqual(self.core.outgoing(1.24), [])
        self.assertEqual(self.core.fatal, 'STOP_ACK_TIMEOUT')

    def test_ack_both_telemetry_orders(self):
        for order in ('lv', 'vl'):
            with self.subTest(order=order):
                self.setUp()
                self.request()
                lines = {'l': b'@LT1,1,10,1,300,0\n', 'v': b'@V1,77,1,0,2,1\n'}
                self.core.ingest_line(lines[order[0]], .3)
                self.assertIsNotNone(self.core.pending)
                self.core.ingest_line(lines[order[1]], .31)
                self.assertIsNone(self.core.pending)
                self.assertEqual(self.core.last_ack['stop_seq'], 10)
                self.assertEqual(self.core.history[-1]['event'], 'stop_confirmed')

    def test_ack_needs_matching_event_class_reason_and_incremented_stop(self):
        for seq, event, cid, reason in ((9, 1, 0, 2), (10, 2, 0, 2), (10, 1, 1, 2), (10, 1, 0, 1)):
            with self.subTest(seq=seq, event=event, cid=cid, reason=reason):
                self.setUp()
                self.request()
                self.telemetry(.3, mode=1, seq=seq, event=event, cid=cid, reason=reason)
                self.assertIsNotNone(self.core.pending)
                self.acknowledge(.6)
                self.assertIsNone(self.core.pending)

    def test_stop_sequence_uint32_wrap(self):
        self.request(seq=U32)
        self.acknowledge(seq=0)
        self.assertEqual(self.core.last_ack['stop_seq'], 0)

    def test_repeated_visible_statue_never_retriggers(self):
        self.request()
        self.acknowledge(mode_after=0)
        for i in range(5, 80):
            self.frame(i/10, seq=10, event=1, cid=0, reason=2)
            self.assertFalse(any(b',S,' in x for x in self.core.outgoing(i/10+.01)))
        self.assertIn(0, self.core.latched)

    def test_continuous_absence_rearms_but_long_frame_gap_does_not(self):
        self.request()
        self.acknowledge(mode_after=0)
        self.frame(.5, [], seq=10)
        self.frame(3., [], seq=10)
        self.assertIn(0, self.core.latched)
        for i in range(31, 46):
            self.frame(i/10, [], seq=10)
        self.assertNotIn(0, self.core.latched)

    def test_cooldown_applies_to_resume_into_every_moving_mode(self):
        for mode in (0, 2, 3):
            with self.subTest(mode=mode):
                self.setUp()
                self.request()
                self.acknowledge(mode_after=mode)
                for i in range(5, 24):
                    self.frame(i/10, [detection(1)], mode=mode, seq=10)
                    self.assertFalse(any(b',S,' in x for x in self.core.outgoing(i/10+.01)))
                self.frame(2.4, [detection(1)], mode=mode, seq=10)
                self.assertIn(b'@VC1,S,77,2,1\n', self.core.outgoing(2.41))

    def test_wrong_session_or_test_mode_cannot_move(self):
        self.frame(0, session=88)
        self.assertEqual(self.core.outgoing(.01), [])
        self.assertIn('SESSION_MISMATCH', self.core.fatal)
        self.setUp()
        self.frame(0)
        self.core.ingest_line(b'@LT1,0,9,0,1,1\n', .01)
        self.assertEqual(self.core.outgoing(.02), [])

    def test_invalid_and_low_confidence_detections_do_not_trigger(self):
        bad = [detection(confidence=.549), detection(confidence=float('nan')),
               {'class_id': [], 'confidence': .8, 'bbox': [0, 0, 1, 1]},
               {'class_id': 0, 'confidence': .8, 'bbox': [1, 0, 0, 1]}]
        for t in (0, .1, .2):
            self.frame(t, bad)
        self.assertFalse(any(b',S,' in x for x in self.core.outgoing(.22)))

    def test_inference_gap_breaks_detection_consensus(self):
        for t in (0, .1, .8):
            self.frame(t)
        self.assertFalse(any(b',S,' in x for x in self.core.outgoing(.81)))

    def test_outside_consensus_can_stop_on_first_current_aligned_frame(self):
        outside = {**detection(), 'stop_eligible': False}
        for i in range(10):
            self.frame(i/10, [outside])
            self.assertFalse(any(b',S,' in x for x in self.core.outgoing(i/10+.02)))
        self.frame(1., [detection()])
        self.assertIn(b'@VC1,S,77,1,0\n', self.core.outgoing(1.02))

    def test_outside_zone_or_display_confidence_still_keeps_class_latched(self):
        for confidence in (.8, .3):
            with self.subTest(confidence=confidence):
                self.setUp()
                self.request()
                self.acknowledge(mode_after=0)
                item = {**detection(confidence=confidence), 'stop_eligible': False}
                for i in range(5, 40):
                    self.frame(i/10, [item], seq=10)
                    self.core.outgoing(i/10+.02)
                self.assertIn(0, self.core.latched)
                for i in range(40, 45):
                    self.frame(i/10, [detection()], seq=10)
                    self.assertFalse(any(b',S,' in x for x in self.core.outgoing(i/10+.02)))


class ProtocolAndPreviewTests(unittest.TestCase):
    def test_sensor_telemetry_requires_exactly_eight_valid_adc_values(self):
        self.assertEqual(parse_sensor_line(b'0\t500\t501\t1023\t2\t3\t4\t5\r\n'),
                         [0, 500, 501, 1023, 2, 3, 4, 5])
        for line in (b'1\t2\t3\n', b'0\t0\t0\t0\t0\t0\t0\t1024\n',
                     b'0\t0\t0\t0\t0\t0\t0\t-1\n', b'@LT1,0,1,0,1,0\n',
                     b'A0\tA1\tA2\tA3\tA4\tA5\tA6\tA7\n'):
            self.assertIsNone(parse_sensor_line(line))

    def test_sensor_status_matches_firmware_threshold_and_reports_age(self):
        hub = VisionHub(VisionStopController(77))
        self.assertIsNone(hub.status()['sensor_values'])
        self.assertIsNone(hub.status()['normal_line'])
        for values, count, normal in (([501, 900, 900, 500, 900, 900, 900, 501], 1, True),
                                      ([900, 900, 900, 900, 0, 900, 900, 900], 1, True),
                                      ([0]*8, 8, False), ([900]*8, 0, False)):
            hub.record_sensors(values, 10.)
            with patch('vision_stop_server.time.monotonic', return_value=10.2):
                state = hub.status()
            self.assertEqual(state['sensor_values'], values)
            self.assertEqual(state['white_count'], count)
            self.assertEqual(state['normal_line'], normal)
            self.assertAlmostEqual(state['sensor_age_seconds'], .2)

    def test_startup_drain_discards_reads_and_resets_buffer_without_writing(self):
        link = Mock()
        stopped = Mock()
        stopped.is_set.return_value = False
        ticks = iter([0, 0, .5, 1., 1.5])
        self.assertTrue(drain_startup_serial(link, stopped, clock=lambda: next(ticks)))
        self.assertEqual(link.read.call_count, 3)
        link.reset_input_buffer.assert_called_once_with()
        link.write.assert_not_called()

    def test_startup_drain_can_stop_without_enabling_the_serial_link(self):
        link, stopped = Mock(), Mock()
        stopped.is_set.return_value = True
        self.assertFalse(drain_startup_serial(link, stopped, clock=lambda: 0))
        link.read.assert_not_called()
        link.reset_input_buffer.assert_not_called()
        link.write.assert_not_called()

    def test_parser_rejects_malformed_out_of_range_and_unknown_frames(self):
        for row in (b'@V1,77,1,8,2,1\n', b'@V1,77,1,0,2,2\n', b'@LT1,6,0,0,1,0\n',
                    b'@LT1,0,0,0,4294967296,0\n', b'@V1,77,+1,0,2,1\n', b'@VC1,H,77\n'):
            self.assertIsNone(parse_line(row))
        self.assertEqual(parse_line(b'@V1,0,0,-1,0,0\r\n')[1]['label'], -1)

    def test_bounded_lines_discard_long_line_and_recover(self):
        framing = BoundedLines()
        self.assertEqual(framing.feed(b'x'*10000), [])
        self.assertLessEqual(len(framing.data), 96)
        self.assertEqual(framing.feed(b'\n@V1,0,0,-1,0,0\n'), [b'@V1,0,0,-1,0,0\n'])

    def test_mjpeg_boundary_and_length(self):
        self.assertEqual(multipart_frame(b'abc'), b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 3\r\n\r\nabc\r\n')

    def test_frame_freshness_uses_acquisition_time(self):
        hub = FrameHub(clock=lambda: 4., stale_seconds=1.)
        hub.publish(b'abc', 480, 640, captured_at=2.)
        self.assertFalse(hub.status()['ok'])
        self.assertEqual(hub.status()['capture_age_seconds'], 2.)
        self.assertIsNone(hub.next_frame(0, timeout=0))
        self.assertTrue(hub.status()['unlimited'])
        self.assertIsNone(hub.status()['remaining_seconds'])

    def handler(self, path, client='100.64.0.10'):
        handler = PreviewHandler.__new__(PreviewHandler)
        handler.connection = SimpleNamespace(settimeout=lambda *_: None)
        handler.client_address = (client, 1234)
        handler.path = path
        hub = FrameHub()
        allowed = frozenset(['100.64.0.10'])
        handler.server = SimpleNamespace(allowed_clients=allowed, hub=hub,
                                        is_client_allowed=lambda value: client_is_allowed(value, allowed))
        handler._send = Mock()
        handler.close_connection = False
        handler.wfile = io.BytesIO()
        return handler

    def test_each_read_endpoint_denies_other_clients_before_read(self):
        for path in ('/', '/status.json', '/snapshot.jpg', '/stream.mjpg'):
            h = self.handler(path, '100.1.2.3')
            h.do_GET()
            self.assertEqual(h._send.call_args.args[0], 403)

    def test_status_response_and_missing_snapshot(self):
        h = self.handler('/status.json')
        h.do_GET()
        status, content_type, body = h._send.call_args.args
        self.assertEqual(status, 200)
        self.assertFalse(json.loads(body)['ok'])
        h.path = '/snapshot.jpg'
        h.do_GET()
        self.assertEqual(h._send.call_args.args[0], 503)

    def test_rotation_is_applied_to_input_and_status_is_serializable(self):
        cv2 = SimpleNamespace(ROTATE_90_COUNTERCLOCKWISE='ccw', ROTATE_180='half',
                              ROTATE_90_CLOCKWISE='cw', rotate=Mock(return_value='rotated'))
        self.assertEqual(rotate_input('frame', 0, cv2), 'frame')
        for angle, code in ((90, 'ccw'), (180, 'half'), (270, 'cw')):
            self.assertEqual(rotate_input('frame', angle, cv2), 'rotated')
            cv2.rotate.assert_called_with('frame', code)
        hub = VisionHub(VisionStopController(77), rotation_ccw=90)
        data = hub.status()
        self.assertEqual(data['rotation_ccw'], 90)
        self.assertEqual(data['version'], 'vision-stop-v1')
        json.dumps(data, allow_nan=False)

    def test_stop_zone_uses_box_center_and_confidence(self):
        item = detection()
        item['bbox'] = [10, 20, 90, 100]
        self.assertTrue(stop_eligible(item, 100, (.4, .6), .55))
        item['bbox'] = [0, 20, 30, 100]
        self.assertFalse(stop_eligible(item, 100, (.4, .6), .55))
        item['bbox'] = [40, 20, 60, 100]
        item['confidence'] = .3
        self.assertFalse(stop_eligible(item, 100, (.4, .6), .55))

    def test_ui_identifies_confidence_and_display_only_pause(self):
        text = PAGE.decode('utf-8')
        self.assertIn('신뢰도', text)
        self.assertIn('AI 제어는 계속됩니다', text)
        self.assertNotIn('100-confidence', text)
        self.assertIn("AbortController", text)





class CenterFastAssociationTests(unittest.TestCase):
    def setUp(self):
        self.core = VisionStopController(77, min_span=.1)
        self.core.set_model_ready(True, 0)
        self.core.set_serial_connected(True, 0)

    def item(self, center=170., width=80., height=180., confidence=.95, cid=0):
        return {'class_id': cid, 'confidence': confidence,
                'bbox': [center-width/2, 100., center+width/2, 100.+height],
                'stop_eligible': .4 <= center/480. <= .6}

    def frame(self, t, items):
        self.core.ingest_line(f'@LT1,0,0,0,{int(t*1000)},0\n'.encode(), t)
        self.core.ingest_line(b'@V1,77,0,-1,0,1\n', t)
        self.core.observe(items, t, t+.001)

    def stops(self, t):
        return [cmd for cmd in self.core.outgoing(t+.002) if b',S,' in cmd]

    def prime_outside(self, center=170.):
        for t in (0., .05, .1):
            self.frame(t, [self.item(center)])
            self.assertEqual(self.stops(t), [])

    def test_smooth_motion_can_trigger_on_third_frame_at_point_one_seconds(self):
        self.frame(0., [self.item(140.)]); self.assertEqual(self.stops(0.), [])
        self.frame(.05, [self.item(170.)]); self.assertEqual(self.stops(.05), [])
        self.frame(.1, [self.item(200.)]); self.assertEqual(self.stops(.1), [b'@VC1,S,77,1,0\n'])

    def test_time_span_and_three_frames_are_both_required(self):
        for t in (0., .025, .099):
            self.frame(t, [self.item(220.)]); self.assertEqual(self.stops(t), [])
        self.frame(.1, [self.item(220.)]); self.assertEqual(len(self.stops(.1)), 1)
        self.setUp()
        for t in (0., .1):
            self.frame(t, [self.item(220.)]); self.assertEqual(self.stops(t), [])
        self.frame(.15, [self.item(220.)]); self.assertEqual(len(self.stops(.15)), 1)

    def test_outside_stability_alone_never_sends_stop(self):
        for i in range(30):
            t=i*.05; self.frame(t, [self.item(170.)]); self.assertEqual(self.stops(t), [])
        self.assertGreater(self.core.streaks[0]['count'], 3)
        self.assertEqual(self.core.eligible, {})

    def test_first_current_aligned_frame_uses_same_object_history(self):
        self.prime_outside(); self.frame(.15, [self.item(200.)])
        self.assertEqual(self.stops(.15), [b'@VC1,S,77,1,0\n'])

    def test_latest_frame_outside_cannot_use_previous_aligned_frame(self):
        self.prime_outside(); self.frame(.15, [self.item(200.)])
        self.frame(.2, [self.item(170.)])
        self.assertEqual(self.stops(.2), [])

    def test_low_confidence_current_center_does_not_stop(self):
        self.prime_outside(); self.frame(.15, [self.item(200., confidence=.54)])
        self.assertEqual(self.stops(.15), [])
        self.assertEqual(self.core.streaks, {})

    def test_disjoint_same_class_center_instance_starts_fresh_consensus(self):
        self.prime_outside(40.)
        self.frame(.15, [self.item(240.)]); self.assertEqual(self.stops(.15), [])
        self.assertEqual(self.core.streaks[0]['count'], 1)
        self.frame(.2, [self.item(240.)]); self.assertEqual(self.stops(.2), [])
        self.frame(.25, [self.item(240.)]); self.assertEqual(len(self.stops(.25)), 1)

    def test_outside_instance_still_visible_cannot_prime_new_center_instance(self):
        self.prime_outside(40.)
        for t in (.15, .2):
            self.frame(t, [self.item(40.), self.item(240.)]); self.assertEqual(self.stops(t), [])
        self.frame(.25, [self.item(40.), self.item(240.)]); self.assertEqual(len(self.stops(.25)), 1)

    def test_alternating_disjoint_center_instances_never_pool_evidence(self):
        for i in range(10):
            t=i*.05; center=200. if i%2 else 280.
            self.frame(t, [self.item(center, width=20.)]); self.assertEqual(self.stops(t), [])
        self.assertEqual(self.core.streaks[0]['count'], 1)

    def test_ambiguous_overlapping_instances_reset_consensus(self):
        self.prime_outside(180.)
        self.frame(.15, [self.item(200.), self.item(210.)]); self.assertEqual(self.stops(.15), [])
        self.assertEqual(self.core.streaks[0]['count'], 1)

    def test_large_size_jump_starts_new_track(self):
        self.prime_outside()
        self.frame(.15, [self.item(200., width=180., height=400.)])
        self.assertEqual(self.stops(.15), [])
        self.assertEqual(self.core.streaks[0]['count'], 1)

    def test_missing_frame_detection_breaks_object_continuity(self):
        self.prime_outside(); self.frame(.15, []); self.frame(.2, [self.item(200.)])
        self.assertEqual(self.stops(.2), [])
        self.assertEqual(self.core.streaks[0]['count'], 1)

    def test_long_frame_gap_breaks_object_continuity(self):
        self.prime_outside(); self.frame(.5, [self.item(200.)])
        self.assertEqual(self.stops(.5), [])
        self.assertEqual(self.core.streaks[0]['count'], 1)

    def test_current_frame_age_still_blocks_stop(self):
        self.prime_outside(); self.frame(.15, [self.item(200.)])
        self.assertEqual(self.stops(1.2), [])


if __name__ == '__main__':
    unittest.main()
