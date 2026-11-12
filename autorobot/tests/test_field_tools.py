"""Field logging and sensor analysis tools. Synthetic data only."""
from pathlib import Path
import json
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import field_log  # noqa: E402
import sensor_capture  # noqa: E402


class FieldLogTests(unittest.TestCase):
    def test_records_validate_and_summarise(self):
        with tempfile.TemporaryDirectory() as d:
            f = str(Path(d) / 'f.jsonl')
            for argv in (['add', 'stop', '--statue', 'Scylla', '--error-mm', '12', '--on-mark', 'yes', '--load', 'deck'],
                         ['add', 'stop', '--statue', 'Scylla', '--error-mm', '40', '--on-mark', 'no'],
                         ['add', 'recognition', '--statue', 'Siren', '--result', 'missed', '--lux', 'dim'],
                         ['add', 'comm', '--error', 'heartbeat_fault'],
                         ['add', 'bump', '--load', 'deck+3', '--result', 'pass'],
                         ['add', 'bump', '--load', 'deck+3', '--result', 'pass'],
                         ['add', 'task', '--task', 'crew_transfer', '--seconds', '14.5', '--crew', '2'],
                         ['add', 'reset', '--cause', 'line_lost'],
                         ['add', 'rehearsal', '--seconds', '290', '--odysseus', 'yes', '--score', '61']):
                self.assertEqual(field_log.main(['--file', f] + argv), 0)
            for bad in (['add', 'stop', '--statue', 'Scylla', '--error-mm', '5'],
                        ['add', 'stop', '--statue', 'S', '--error-mm', '-1', '--on-mark', 'yes'],
                        ['add', 'stop', '--statue', 'S', '--error-mm', '1', '--on-mark', 'maybe'],
                        ['add', 'comm', '--error', 'gremlins']):
                with self.assertRaises(SystemExit):
                    field_log.main(['--file', f] + bad)
            text = field_log.summarize(field_log.read_rows(f))
            self.assertIn('on-mark 1/2', text)
            self.assertIn('missed=1', text)
            self.assertIn('current=2 best=2', text)
            self.assertIn('odysseus 1/1', text)
            self.assertEqual(len(field_log.read_rows(f)), 9)

    def test_import_mission_log(self):
        rows = [{'mission': {'event': 'stop_confirmed', 'trial': 1, 'seek_s': 7.5}},
                {'mission': {'event': 'crew_done', 'trial': 1, 'wait_s': 11.0}},
                {'mission': {'event': 'motion', 'motion': 'DEPARTING', 'trial': 1, 'dwell_s': 13.0}},
                {'mission': {'event': 'motion', 'motion': 'FAULT', 'reason': 'MEGA_FAULT_LINK_FAULT'}}]
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'm.jsonl'
            p.write_text(''.join('2026-11-12 10:00:00 INFO ' + json.dumps(r) + '\n' for r in rows) + 'garbage\n')
            got = field_log.import_mission(p)
        self.assertEqual([r['task'] for r in got if r['type'] == 'task'],
                         ['seek_to_statue', 'stop_to_crew_done', 'dwell_at_statue'])
        self.assertEqual(got[-1]['note'], 'MEGA_FAULT_LINK_FAULT')


class SensorTests(unittest.TestCase):
    def test_parse_and_gap_analysis(self):
        self.assertEqual(sensor_capture.parse_row('1\t2\t3\t4\t5\t6\t7\t8\n'), [1, 2, 3, 4, 5, 6, 7, 8])
        for bad in ('1\t2', '@LT1,0,0,0,0,0', '1\t2\t3\t4\t5\t6\t7\t9999'):
            self.assertIsNone(sensor_capture.parse_row(bad))
        line, dark = [800, 800, 800, 60, 70, 800, 800, 800], [800] * 8
        rows = [(i * .1, line) for i in range(5)] + [(.5 + i * .1, dark) for i in range(3)] + [(.8, line), (.9, [60] * 8)]
        result = sensor_capture.analyze(rows)
        self.assertEqual(result['all_dark_gaps_s'], [0.3])
        self.assertEqual(result['all_white_samples'], 1)
        self.assertEqual(result['white_count_histogram'], {0: 3, 2: 6, 8: 1})
        a3 = result['per_channel'][3]
        self.assertEqual((a3['white_max'], a3['dark_min']), (60, 800))
        self.assertEqual(sensor_capture.analyze([])['samples'], 0)


if __name__ == '__main__':
    unittest.main()
