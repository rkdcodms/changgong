"""Access policy tests; no sockets, serial ports, camera or motors are opened."""
from contextlib import redirect_stderr
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'jetson'))
from vision_stop_server import parse_args, DEFAULT_WEIGHTS
from web_preview import PreviewServer, PreviewHandler, FrameHub, TAILSCALE_NETWORK


class AccessPolicyTests(unittest.TestCase):
    def server(self, clients=(), networks=()):
        with patch('web_preview.ThreadingHTTPServer.__init__', return_value=None):
            return PreviewServer(('100.64.0.1', 8765), FrameHub(), clients, networks)

    def handler(self, server, address, path='/status.json'):
        handler = PreviewHandler.__new__(PreviewHandler)
        handler.connection = SimpleNamespace(settimeout=lambda *_: None)
        handler.client_address = (address, 12345)
        handler.path = path
        handler.server = server
        handler.close_connection = False
        handler.wfile = io.BytesIO()
        handler._send = Mock()
        return handler

    def rejected(self, *arguments):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            parse_args(list(arguments))
        self.assertEqual(error.exception.code, 2)

    def test_new_tailscale_clients_need_no_registration(self):
        server = self.server(networks=[TAILSCALE_NETWORK])
        for peer in ('100.64.0.0', '100.64.0.1', '100.80.10.20', '100.127.255.254', '100.127.255.255'):
            handler = self.handler(server, peer)
            handler.do_GET()
            self.assertEqual(handler._send.call_args.args[0], 200)

    def test_non_tailscale_peers_are_denied_at_every_read_endpoint(self):
        server = self.server(networks=[TAILSCALE_NETWORK])
        for peer in ('100.63.255.255', '100.128.0.0', '192.168.1.10', '127.0.0.1', '8.8.8.8'):
            for path in ('/', '/status.json', '/snapshot.jpg', '/stream.mjpg'):
                handler = self.handler(server, peer, path)
                handler.do_GET()
                self.assertEqual(handler._send.call_args.args[0], 403, (peer, path))

    def test_explicit_policy_allows_multiple_viewers_and_rejects_others(self):
        server = self.server(clients=['100.64.0.10', '100.64.0.20'])
        for peer in ('100.64.0.10', '100.64.0.20'):
            handler = self.handler(server, peer)
            handler.do_GET()
            self.assertEqual(handler._send.call_args.args[0], 200)
        handler = self.handler(server, '100.64.0.30')
        handler.do_GET()
        self.assertEqual(handler._send.call_args.args[0], 403)

    def test_explicit_viewer_and_tailscale_policy_can_be_combined(self):
        server = self.server(clients=['127.0.0.1'], networks=[TAILSCALE_NETWORK])
        self.assertTrue(server.is_client_allowed('127.0.0.1'))
        self.assertTrue(server.is_client_allowed('100.64.0.123'))
        self.assertFalse(server.is_client_allowed('192.168.1.1'))

    def test_loopback_only_policy_does_not_allow_tailscale_viewers(self):
        server = self.server(clients=['127.0.0.1'])
        self.assertTrue(server.is_client_allowed('127.0.0.1'))
        handler = self.handler(server, '100.64.0.10')
        handler.do_GET()
        self.assertEqual(handler._send.call_args.args[0], 403)

    def test_forwarded_header_does_not_override_socket_peer(self):
        server = self.server(networks=[TAILSCALE_NETWORK])
        handler = self.handler(server, '192.168.1.10')
        handler.headers = {'X-Forwarded-For': '100.64.0.10'}
        handler.do_GET()
        self.assertEqual(handler._send.call_args.args[0], 403)

    def test_bad_addresses_fail_closed(self):
        server = self.server(networks=[TAILSCALE_NETWORK])
        for peer in ('::1', '::ffff:100.64.0.10', '100.64.0.10.bad', '', 'not-an-ip'):
            self.assertFalse(server.is_client_allowed(peer))

    def test_cidr_subset_works_without_widening_to_other_addresses(self):
        server = self.server(networks=['100.64.1.0/24'])
        self.assertTrue(server.is_client_allowed('100.64.1.4'))
        self.assertFalse(server.is_client_allowed('100.64.2.4'))

    def test_server_rejects_empty_or_overbroad_policy_before_binding(self):
        for networks in ([], ['0.0.0.0/0'], ['100.0.0.0/8'], ['192.168.1.0/24']):
            with self.assertRaises(ValueError):
                self.server(networks=networks)

    def test_no_write_endpoint_and_four_stream_slots_preserved(self):
        server = self.server(networks=[TAILSCALE_NETWORK])
        for _ in range(4):
            self.assertTrue(server.stream_slots.acquire(blocking=False))
        self.assertFalse(server.stream_slots.acquire(blocking=False))
        for _ in range(4):
            server.stream_slots.release()
        for verb in ('POST', 'PUT', 'DELETE', 'PATCH'):
            self.assertFalse(hasattr(PreviewHandler, 'do_' + verb))

    def test_cli_accepts_team_mode_with_tailscale_bind(self):
        args = parse_args(['--bind', '100.64.0.1', '--serial', 'auto', '--allow-tailscale'])
        self.assertTrue(args.allow_tailscale)
        self.assertEqual(args.allow_client, [])
        self.assertEqual((args.stop_zone_left, args.stop_zone_right, args.confidence), (.4, .6, .55))
        self.assertEqual(args.max_seconds, 0)
        self.assertEqual(DEFAULT_WEIGHTS, Path(__file__).resolve().parents[1] / 'models/best.pt')

    def test_cli_accepts_repeated_explicit_clients_and_loopback_bind(self):
        args = parse_args(['--bind', '127.0.0.1', '--serial', 'auto',
                           '--allow-client', '127.0.0.1', '--allow-client', '100.64.0.10'])
        self.assertEqual(args.allow_client, ['127.0.0.1', '100.64.0.10'])
        self.assertFalse(args.allow_tailscale)

    def test_cli_requires_serial_and_a_viewer_policy(self):
        self.rejected('--bind', '100.64.0.1', '--serial', 'auto')
        self.rejected('--bind', '100.64.0.1', '--allow-tailscale')

    def test_team_mode_rejects_loopback_wildcard_lan_and_ipv6_binds(self):
        for bind in ('127.0.0.1', '0.0.0.0', '192.168.1.10', '8.8.8.8', '::1'):
            self.rejected('--bind', bind, '--serial', 'auto', '--allow-tailscale')

    def test_cli_rejects_invalid_explicit_client(self):
        self.rejected('--bind', '100.64.0.1', '--serial', 'auto', '--allow-client', '100.64.0.10/24')


if __name__ == '__main__':
    unittest.main()
