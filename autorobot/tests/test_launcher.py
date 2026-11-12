"""No hardware/network: configuration, startup boundary and model-download checks."""
from __future__ import annotations

from contextlib import redirect_stdout, redirect_stderr
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load_script(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT/'scripts'/filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


run = load_script('team_run_robot_test', 'run_robot.py')
download = load_script('team_download_model_test', 'download_model.py')


def config():
    # Synthetic Tailscale-range address; not a real team machine.
    return {
        'bind': '100.64.0.10', 'port': 8765,
        'allow_tailscale': True, 'allowed_clients': [],
        'camera': '/dev/video-test', 'rotate_ccw': 90,
        'serial': '/dev/serial/by-id/example-mega',
        'weights': 'models/best.pt', 'confidence': .55,
        'stop_zone_left': .4, 'stop_zone_right': .6, 'max_seconds': 0,
    }


class ConfigurationTests(unittest.TestCase):
    def test_example_requires_real_bind_and_serial_before_start(self):
        sample = json.loads((ROOT/'config/robot.example.json').read_text())
        with self.assertRaises(ValueError):
            run.validate(sample)
        sample['bind'] = config()['bind']
        with self.assertRaises(ValueError):
            run.validate(sample)

    def test_relative_model_path_is_relative_to_package_not_calling_directory(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            value = config()
            result = run.validate(value, root)
            self.assertEqual(result['weights'], str((root/'models/best.pt').resolve()))
            self.assertEqual(value['weights'], 'models/best.pt')

    def test_public_wildcard_lan_and_ipv6_binds_are_rejected(self):
        for address in ('0.0.0.0', '192.0.2.5', '192.168.1.20', '::', '::1'):
            with self.subTest(address=address), self.assertRaises(ValueError):
                run.validate({**config(), 'bind': address})

    def test_loopback_needs_explicit_policy_instead_of_tailnet_wide_flag(self):
        value = {**config(), 'bind': '127.0.0.1'}
        with self.assertRaises(ValueError):
            run.validate(value)
        result = run.validate({**value, 'allow_tailscale': False, 'allowed_clients': ['127.0.0.1']})
        self.assertEqual(result['bind'], '127.0.0.1')

    def test_missing_empty_policy_and_unknown_keys_fail_closed(self):
        for bad in ({**config(), 'allow_tailscale': False},
                    {**config(), 'unexpected_setting': 1},
                    {k: v for k, v in config().items() if k != 'confidence'},
                    []):
            with self.subTest(value=bad), self.assertRaises(ValueError):
                run.validate(bad)

    def test_invalid_numbers_and_types_are_rejected(self):
        for name, bad in [('port', True), ('port', 0), ('port', 65536), ('port', 8765.5),
                          ('max_seconds', -1), ('max_seconds', True), ('rotate_ccw', 45),
                          ('confidence', 0), ('confidence', 1.01), ('confidence', float('nan')),
                          ('stop_zone_left', float('inf')), ('allow_tailscale', 'true')]:
            with self.subTest(name=name, bad=bad), self.assertRaises(ValueError):
                run.validate({**config(), name: bad})

    def test_reversed_or_zero_width_stop_zone_is_rejected(self):
        for left, right in ((.6, .4), (.5, .5), (-.1, .6), (.4, 1.1)):
            with self.subTest(left=left, right=right), self.assertRaises(ValueError):
                run.validate({**config(), 'stop_zone_left': left, 'stop_zone_right': right})

    def test_client_list_rejects_cidr_hostname_nonstring_and_wrong_container(self):
        for bad in (['100.64.0.0/10'], ['example.invalid'], [123], '100.64.0.20', None):
            with self.subTest(value=bad), self.assertRaises(ValueError):
                run.validate({**config(), 'allowed_clients': bad})

    def test_camera_serial_and_weights_reject_missing_or_unsafe_path_types(self):
        for key, value in [('camera', ''), ('camera', 'https://example.invalid/stream'),
                           ('serial', 'auto'), ('serial', '/dev/REPLACE_WITH_PORT'),
                           ('serial', '/dev/example\x00'), ('weights', ''), ('weights', None)]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                run.validate({**config(), key: value})

    def test_command_keeps_tuned_values_and_all_clients_as_distinct_arguments(self):
        with tempfile.TemporaryDirectory(prefix='robot path ') as d:
            root = Path(d)
            c = run.validate({**config(), 'allowed_clients': ['100.64.0.20', '100.64.0.21']}, root)
            command = run.command(c, executable='example-python', root=root)
            def argument(name):
                return command[command.index(name) + 1]
            self.assertEqual(command[:2], ['example-python', '-u'])
            self.assertIn('--allow-tailscale', command)
            self.assertEqual([command[i + 1] for i, value in enumerate(command) if value == '--allow-client'],
                             ['100.64.0.20', '100.64.0.21'])
            self.assertEqual(argument('--confidence'), '0.55')
            self.assertEqual(argument('--rotate-ccw'), '90')
            self.assertEqual(argument('--stop-zone-left'), '0.4')
            self.assertEqual(argument('--stop-zone-right'), '0.6')
            self.assertEqual(argument('--max-seconds'), '0')
            self.assertEqual(argument('--weights'), str((root/'models/best.pt').resolve()))
            self.assertEqual(argument('--status-file'), str(root/'runtime/runtime_status.json'))
            self.assertEqual(argument('--log-file'), str(root/'runtime/vision_events.jsonl'))

    def test_explicit_client_only_command_does_not_enable_whole_tailnet(self):
        c = run.validate({**config(), 'allow_tailscale': False, 'allowed_clients': ['100.64.0.20']})
        command = run.command(c)
        self.assertNotIn('--allow-tailscale', command)
        self.assertEqual(command.count('--allow-client'), 1)

    def test_utf8_bom_local_configuration_loads(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'config.json'
            path.write_text(json.dumps(config()), encoding='utf-8-sig')
            self.assertEqual(run.load_config(path)['port'], 8765)


class StartupBoundaryTests(unittest.TestCase):
    def test_check_only_reads_config_and_model_never_devices_or_executes(self):
        body = b'test model bytes, never a serialized model'
        with tempfile.TemporaryDirectory() as d:
            root = Path(d).resolve()
            model = root/'model.pt'
            model.write_bytes(body)
            local = root/'config.json'
            local.write_text(json.dumps({**config(), 'weights': str(model)}), encoding='utf-8')
            opened = []
            original_open = Path.open
            def track(path, *args, **kwargs):
                opened.append(path)
                self.assertIn(path, (local, model), 'Only configuration/model file reads are allowed')
                return original_open(path, *args, **kwargs)
            with patch.object(run, 'WEIGHTS_HASH', hashlib.sha256(body).hexdigest()), \
                 patch.object(Path, 'open', autospec=True, side_effect=track), \
                 patch.object(run.os, 'execv', side_effect=AssertionError('must not execute')) as execute, \
                 patch.object(run.os, 'chdir', side_effect=AssertionError('must not change runtime cwd')), \
                 patch.object(run.stat, 'S_ISCHR', side_effect=AssertionError('must not inspect devices')), \
                 redirect_stdout(io.StringIO()):
                self.assertEqual(run.main(['--config', str(local), '--check']), 0)
            self.assertEqual(opened, [local, model])
            execute.assert_not_called()
            self.assertFalse((root/'runtime').exists())

    def test_start_rejects_laptop_before_device_open_or_exec(self):
        with patch.object(run, 'load_config', return_value=run.validate(config())), \
             patch.object(run, 'check_model'), patch.object(run.sys, 'platform', 'win32'), \
             patch.object(run.stat, 'S_ISCHR', side_effect=AssertionError('must not inspect devices')), \
             patch.object(run.os, 'execv', side_effect=AssertionError('must not execute')), \
             redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as result:
                run.main(['--start'])
            self.assertEqual(result.exception.code, 1)

    def test_explicit_check_or_start_is_required(self):
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as result:
                run.main([])
            self.assertEqual(result.exception.code, 2)

    def test_hash_mismatch_blocks_start_before_execution(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'different.pt'
            path.write_bytes(b'wrong model')
            with self.assertRaisesRegex(ValueError, 'SHA-256'):
                run.check_model(path)

    def test_check_model_rejects_nonregular_path_without_opening_it(self):
        # A mocked nonregular path represents a character device; never touch one.
        with patch.object(Path, 'is_file', return_value=False), \
             patch.object(Path, 'open', side_effect=AssertionError('must not open a device')):
            with self.assertRaises(ValueError):
                run.check_model('/not-a-real-device')


class ModelDownloadTests(unittest.TestCase):
    body = b'small deterministic model fixture; not executable'

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.target = Path(self.directory.name)/'best.pt'
        self.patch_size = patch.object(download, 'SIZE', len(self.body))
        self.patch_hash = patch.object(download, 'SHA256', hashlib.sha256(self.body).hexdigest())
        self.patch_size.start()
        self.patch_hash.start()
        self.addCleanup(self.patch_size.stop)
        self.addCleanup(self.patch_hash.stop)

    def test_existing_correct_model_is_reused_without_network(self):
        self.target.write_bytes(self.body)
        with patch.object(download.urllib.request, 'urlopen', side_effect=AssertionError('no network')), \
             redirect_stdout(io.StringIO()):
            download.download(self.target)
        self.assertEqual(self.target.read_bytes(), self.body)

    def test_existing_wrong_model_is_preserved(self):
        self.target.write_bytes(b'teammate model')
        with patch.object(download.urllib.request, 'urlopen', side_effect=AssertionError('no network')):
            with self.assertRaisesRegex(RuntimeError, 'different model'):
                download.download(self.target)
        self.assertEqual(self.target.read_bytes(), b'teammate model')

    def test_verified_download_installs_exact_bytes_and_cleans_temp(self):
        with patch.object(download.urllib.request, 'urlopen', return_value=io.BytesIO(self.body)) as fetch, \
             redirect_stdout(io.StringIO()):
            download.download(self.target)
        self.assertEqual(self.target.read_bytes(), self.body)
        self.assertEqual(list(self.target.parent.glob('*.download')), [])
        request = fetch.call_args.args[0]
        self.assertEqual(request.full_url, download.URL)

    def test_truncated_oversized_and_same_size_wrong_downloads_are_not_installed(self):
        bad_payloads = (self.body[:-1], self.body+b'X', b'x'*len(self.body))
        for bad in bad_payloads:
            with self.subTest(size=len(bad)), \
                 patch.object(download.urllib.request, 'urlopen', return_value=io.BytesIO(bad)):
                with self.assertRaises(RuntimeError):
                    download.download(self.target)
            self.assertFalse(self.target.exists())
            self.assertEqual(list(self.target.parent.glob('*.download')), [])

    def test_network_failure_leaves_no_destination_or_temporary_file(self):
        with patch.object(download.urllib.request, 'urlopen', side_effect=OSError('simulated network failure')):
            with self.assertRaises(OSError):
                download.download(self.target)
        self.assertFalse(self.target.exists())
        self.assertEqual(list(self.target.parent.glob('*.download')), [])

    def test_concurrently_created_destination_is_not_overwritten(self):
        def create_during_download(*args, **kwargs):
            self.target.write_bytes(b'teammate saved this')
            return io.BytesIO(self.body)
        with patch.object(download.urllib.request, 'urlopen', side_effect=create_during_download):
            with self.assertRaises(FileExistsError):
                download.download(self.target)
        self.assertEqual(self.target.read_bytes(), b'teammate saved this')
        self.assertEqual(list(self.target.parent.glob('*.download')), [])

    def test_atomic_install_failure_leaves_no_partial_destination(self):
        with patch.object(download.urllib.request, 'urlopen', return_value=io.BytesIO(self.body)), \
             patch.object(download.os, 'link', side_effect=OSError('simulated filesystem failure')):
            with self.assertRaises(OSError):
                download.download(self.target)
        self.assertFalse(self.target.exists())
        self.assertEqual(list(self.target.parent.glob('*.download')), [])


if __name__ == '__main__':
    unittest.main()
