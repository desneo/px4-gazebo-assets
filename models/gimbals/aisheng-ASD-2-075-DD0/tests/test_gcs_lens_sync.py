#!/usr/bin/env python3
"""镜头遥测同步、手动切换确认及自动回广角回归测试。"""
from __future__ import annotations

from unittest import TestCase, main
from unittest.mock import Mock
from test_gcs_ui import GCS


class LensSyncTest(TestCase):
    def station(self, active="tele", pending=False):
        station = GCS.GCS.__new__(GCS.GCS)
        station.active = active
        station.pending_video_restart = pending
        station.telemetry_overlay = object()
        station._recreate_video = Mock()
        station.last_cmd = ""
        return station

    def test_automatic_wide_follows_telemetry_once(self):
        station = self.station()
        frame = GCS.proto.TelemetryFrame(status_control2=1)
        self.assertTrue(station._sync_lens(frame))
        self.assertEqual(station.active, "wide")
        self.assertFalse(station.pending_video_restart)
        self.assertIsNone(station.telemetry_overlay)
        self.assertFalse(station._sync_lens(frame))
        station._recreate_video.assert_called_once()

    def test_manual_request_waits_for_matching_telemetry(self):
        station = self.station(pending=True)
        self.assertFalse(station._sync_lens(GCS.proto.TelemetryFrame(status_control2=1)))
        self.assertEqual(station.active, "tele")
        station._recreate_video.assert_not_called()
        self.assertTrue(station._sync_lens(GCS.proto.TelemetryFrame(status_control2=0x11)))
        station._recreate_video.assert_called_once()
        self.assertFalse(station.pending_video_restart)

    def test_no_telemetry_does_not_confirm_pending_wide(self):
        station = self.station(active="wide", pending=True)
        self.assertFalse(station._sync_lens(None))
        self.assertTrue(station.pending_video_restart)
        station._recreate_video.assert_not_called()


if __name__ == "__main__":
    main(verbosity=2)
