#!/usr/bin/env python3
"""A2416A H.264硬件视频契约测试。"""
from __future__ import annotations

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from video import hardware
from video.hardware import HardwareVideoError
from video.rtsp_server import INGRESS_LAUNCH, LAUNCH
from video.receiver import receiver_pipeline
from video.sender import DEFAULT_BITRATE_KBPS, FPS, HEIGHT, WIDTH, encoder_pipeline


class EmptyFactory:
    @staticmethod
    def find(_name):
        return None


class EmptyGst:
    ElementFactory = EmptyFactory


class VideoTest(unittest.TestCase):
    def test_missing_nvcodec_fails_without_software_fallback(self):
        original = hardware.Gst
        hardware.Gst = EmptyGst
        try:
            with self.assertRaisesRegex(HardwareVideoError, "不允许x264enc软件回退"):
                hardware.require_nvcodec(need_encoder=True, need_decoder=False)
            with self.assertRaisesRegex(HardwareVideoError, "nvh264dec"):
                hardware.require_nvcodec(need_encoder=False, need_decoder=True)
        finally:
            hardware.Gst = original

    def test_sender_contract_is_h264_1080p25_2mbps(self):
        launch = encoder_pipeline("nvcudah264enc", "127.0.0.1", 7001,
                                  DEFAULT_BITRATE_KBPS)
        self.assertEqual((WIDTH, HEIGHT, FPS, DEFAULT_BITRATE_KBPS), (1920, 1080, 25, 2000))
        self.assertIn("video/x-raw,format=RGB,width=1920,height=1080,framerate=25/1", launch)
        self.assertIn("video/x-raw,format=NV12", launch)
        self.assertIn("nvcudah264enc", launch)
        self.assertIn("bitrate=2000", launch)
        self.assertIn("video/x-h264,profile=high,stream-format=byte-stream,alignment=au", launch)
        self.assertIn("rtph264pay", launch)
        self.assertNotIn("x264enc", launch)

    def test_rtsp_republishes_only_h264_rtp(self):
        self.assertIn("encoding-name=H264", INGRESS_LAUNCH)
        self.assertIn("rtph264depay", INGRESS_LAUNCH)
        self.assertNotIn("max-size-buffers=3", INGRESS_LAUNCH)
        self.assertIn("shmsink", INGRESS_LAUNCH)
        self.assertIn("shmsrc", LAUNCH)
        self.assertIn("rtph264pay", LAUNCH)
        self.assertNotIn("H265", INGRESS_LAUNCH + LAUNCH)

    def test_receiver_forces_nvidia_h264_decode(self):
        launch = receiver_pipeline(
            "rtsp://127.0.0.1:8554/jielian_a2416a", "nvh264dec", 960, 540)
        self.assertIn("protocols=tcp", launch)
        self.assertNotIn("drop-on-latency=true", launch)
        self.assertIn("rtph264depay ! h264parse ! nvh264dec", launch)
        self.assertIn("appsink name=sink emit-signals=true", launch)
        self.assertNotIn("avdec_h264", launch)
        self.assertNotIn("openh264dec", launch)


if __name__ == "__main__":
    unittest.main()
