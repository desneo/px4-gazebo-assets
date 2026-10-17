#!/usr/bin/env python3
"""实际 NVENC/RTSP 兼容性测试；需要 NVIDIA 设备、GStreamer 与 ffprobe。"""
import json
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest

SERVICE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVICE))
from rtsp_relay import request

WIDTH, HEIGHT = 640, 480


def worker(port):
    import video
    stream = video.VideoStreamer(WIDTH, HEIGHT, rtsp_port=port,
                                 mount='/video_compatibility_1')
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    image = bytes((240, 40, 20)) * (WIDTH * HEIGHT)
    try:
        sequence = 0
        while running:
            stream.push(sequence, image, time.monotonic())
            sequence += 1
            time.sleep(1 / 30)
    finally:
        stream.close()


@unittest.skipUnless(shutil.which('ffprobe'), '需要 ffprobe')
class NvencCompatibilityTests(unittest.TestCase):
    def setUp(self):
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            self.port = reservation.getsockname()[1]
        self.log = tempfile.TemporaryFile()
        self.addCleanup(self.log.close)
        self.processes = []
        self.addCleanup(self.stop_processes)
        self.processes.append(subprocess.Popen(
            [sys.executable, str(SERVICE / 'rtsp_relay.py'), '--port', str(self.port)],
            stdout=self.log, stderr=self.log))
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            try:
                request(self.port, {'op': 'ping'})
                break
            except (OSError, ValueError, RuntimeError):
                time.sleep(.1)
        else:
            self.fail('独立 RTSP 中继启动超时')

    def stop_processes(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)

    def test_production_nvenc_is_8bit_420_at_30fps(self):
        process = subprocess.Popen(
            [sys.executable, __file__, '--worker', str(self.port)],
            stdout=self.log, stderr=self.log)
        self.processes.append(process)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            self.assertIsNone(process.poll(), 'NVENC 编码进程提前退出')
            if '/video_compatibility_1' in request(self.port, {'op': 'ping'})['paths']:
                break
            time.sleep(.1)
        else:
            self.fail('视频路径注册超时')
        url = f'rtsp://127.0.0.1:{self.port}/video_compatibility_1'
        result = subprocess.run(
            ['ffprobe', '-v', 'error', '-rtsp_transport', 'tcp', '-i', url,
             '-read_intervals', '%+2', '-select_streams', 'v:0', '-count_frames',
             '-show_entries',
             'stream=codec_name,profile,pix_fmt,width,height,avg_frame_rate,nb_read_frames',
             '-of', 'json'], capture_output=True, text=True, timeout=20, check=True)
        stream = json.loads(result.stdout)['streams'][0]
        self.assertEqual(stream['codec_name'], 'h264')
        self.assertEqual((stream['width'], stream['height']), (WIDTH, HEIGHT))
        self.assertEqual(stream['pix_fmt'], 'yuv420p', stream)
        self.assertNotIn('4:4:4', stream['profile'], stream)
        self.assertGreaterEqual(int(stream['nb_read_frames']), 40, stream)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--worker':
        worker(int(sys.argv[2]))
    else:
        unittest.main()
