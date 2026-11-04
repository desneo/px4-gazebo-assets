"""中科UDP租约、受限MANUAL watchdog与坏包边界。"""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from common import protocol as proto
from controller.service.udp_session import UdpSession


class SessionTest(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.stops = []
        self.session = UdpSession(clock=lambda: self.now, on_stop=self.stops.append)
        self.peer = ('127.0.0.1', 41001)
        self.other = ('127.0.0.1', 41002)

    def receive(self, cmd=proto.CMD_NOP, x=0, y=0, *, peer=None):
        return self.session.receive(proto.encode_command(cmd, x, y), peer or self.peer)

    def test_first_valid_packet_owns_single_lease(self):
        self.assertIsNotNone(self.receive())
        self.assertIsNone(self.receive(peer=self.other))
        self.assertEqual(self.session.telemetry_peer(), self.peer)
        self.assertEqual(self.session.last_seen, 0.0)

    def test_invalid_packets_cannot_redirect_or_renew(self):
        self.receive()
        self.now = 1.0
        for peer in (self.peer, self.other):
            self.assertIsNone(self.session.receive(b'{"cmd":"ping"}', peer))
            self.assertIsNone(self.session.receive(b'\xee\x16' + bytes(30), peer))
        unsupported = bytearray(proto.encode_command(proto.CMD_NOP))
        unsupported[2] = 0x3A
        unsupported[15] = sum(unsupported[:15]) & 255
        self.assertIsNone(self.session.receive(unsupported, self.peer))
        self.assertEqual(self.session.last_seen, 0.0)
        self.assertEqual(self.session.telemetry_peer(), self.peer)

    def test_unimplemented_valid_command_does_not_establish_session(self):
        session = UdpSession(accepted_commands={proto.CMD_NOP, proto.CMD_MANUAL})
        self.assertIsNone(session.receive(proto.encode_command(proto.CMD_AUTO_LOCK), self.peer))
        self.assertIsNone(session.peer)
        self.assertIn('未实现', session.last_error)

    def test_expired_lease_stops_once_and_can_be_acquired_by_new_peer(self):
        self.receive(proto.CMD_AUTO_LOCK)
        self.now = 2.0
        self.assertEqual(self.session.poll(), 'peer_timeout')
        self.assertEqual(self.stops, ['peer_timeout'])
        self.assertIsNone(self.session.telemetry_peer())
        self.assertIsNone(self.session.poll())
        self.assertIsNotNone(self.receive(peer=self.other))
        self.assertEqual(self.session.peer, self.other)

    def test_heartbeat_extends_lease_without_refreshing_manual_watchdog(self):
        self.receive(proto.CMD_MANUAL, 200, -100)
        self.now = 0.2
        self.receive()
        self.assertEqual(self.session.last_manual, 0.0)
        self.now = 0.5
        self.assertEqual(self.session.poll(), 'manual_timeout')
        self.assertEqual(self.stops, ['manual_timeout'])
        self.assertEqual(self.session.peer, self.peer)
        self.assertIsNone(self.session.poll())

    def test_manual_refresh_prevents_manual_timeout(self):
        self.receive(proto.CMD_MANUAL, 200)
        self.now = 0.4
        self.receive(proto.CMD_MANUAL, 100)
        self.now = 0.7
        self.assertIsNone(self.session.poll())
        self.assertEqual(self.session.mode, 'MANUAL')

    def test_zero_manual_command_cancels_old_watchdog(self):
        self.receive(proto.CMD_MANUAL, 200)
        self.now = 0.1
        self.receive(proto.CMD_MANUAL)
        self.now = 0.7
        self.assertIsNone(self.session.poll())
        self.assertFalse(self.stops)
        self.assertEqual(self.session.mode, 'IDLE')

    def test_new_home_track_auto_angle_or_cancel_is_not_stopped_by_old_manual(self):
        for cmd in (proto.CMD_HOME, proto.CMD_POINT_TRACK, proto.CMD_AUTO_LOCK,
                    proto.CMD_FRAME_ANGLE, proto.CMD_STOP_TRACK):
            with self.subTest(cmd=cmd):
                self.setUp()
                self.receive(proto.CMD_MANUAL, 200)
                self.now = 0.1
                self.receive(cmd)
                self.now = 0.7
                self.assertIsNone(self.session.poll())
                self.assertFalse(self.stops)
                self.assertIsNone(self.session.last_manual)

    def test_lease_timeout_still_stops_new_home_mode(self):
        self.receive(proto.CMD_MANUAL, 200)
        self.now = 0.1
        self.receive(proto.CMD_HOME)
        self.now = 2.1
        self.assertEqual(self.session.poll(), 'peer_timeout')
        self.assertEqual(self.stops, ['peer_timeout'])

    def test_timeouts_reject_invalid_configuration(self):
        for values in ((0, .5), (2, -1), (float('nan'), .5), (2, True)):
            with self.assertRaises(ValueError):
                UdpSession(*values)


if __name__ == '__main__':
    unittest.main()
