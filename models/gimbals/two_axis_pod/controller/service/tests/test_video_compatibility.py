#!/usr/bin/env python3
"""实际 x264/RTSP 兼容性测试；需要 GStreamer、ffprobe 和 ffmpeg。"""
import ast
import json
from pathlib import Path
import re
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


def bridge_pipeline():
    """读取生产常量，避免测试进程导入 Gazebo 消息绑定。"""
    tree = ast.parse((SERVICE / 'gimbal_bridge.py').read_text())
    assignment = next(node for node in tree.body if isinstance(node, ast.Assign)
                      and any(isinstance(target, ast.Name)
                              and target.id == 'X264_ENCODE_LAUNCH'
                              for target in node.targets))
    return ast.literal_eval(assignment.value)


def worker(port, legacy):
    import video
    pipeline = bridge_pipeline()
    if legacy:
        # 重建没有限定色度格式的管线，确认测试可检出 RGB 协商出的 4:4:4。
        pipeline, removed = re.subn(r'!\s*video/x-raw,format=I420\s*', '', pipeline)
        if removed != 1:
            raise RuntimeError('无法构造未限定色度格式的对照管线')
    video.ENCODE_LAUNCH = pipeline
    stream = video.VideoStreamer(WIDTH, HEIGHT, rtsp_port=port,
                                 mount='/video_compatibility_1')
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    # 高饱和 RGB 图像让未限定的转换自动协商 4:4:4；帧序号继续由生产代码烧录。
    image = bytes((240, 40, 20)) * (WIDTH * HEIGHT)
    try:
        sequence = 0
        while running:
            stream.push(sequence, image, time.monotonic())
            sequence += 1
            time.sleep(1 / 30)
    finally:
        stream.close()


@unittest.skipUnless(shutil.which('ffprobe') and shutil.which('ffmpeg'),
                     '需要 ffprobe 和 ffmpeg')
class X264CompatibilityTests(unittest.TestCase):
    def setUp(self):
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            self.port = reservation.getsockname()[1]
        self.log = tempfile.TemporaryFile()
        self.addCleanup(self.log.close)
        # 中继和发送进程均由本测试独立持有，清理不会触及正在运行的 8555。
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

    def probe(self, legacy=False):
        process = subprocess.Popen(
            [sys.executable, __file__, '--worker', str(self.port), str(int(legacy))],
            stdout=self.log, stderr=self.log)
        self.processes.append(process)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            self.assertIsNone(process.poll(), '编码进程提前退出')
            if '/video_compatibility_1' in request(self.port, {'op': 'ping'})['paths']:
                break
            time.sleep(.1)
        else:
            self.fail('视频路径注册超时')
        url = f'rtsp://127.0.0.1:{self.port}/video_compatibility_1'
        result = subprocess.run(
            ['ffprobe', '-v', 'error', '-rtsp_transport', 'tcp', '-i', url,
             '-read_intervals', '%+2', '-select_streams', 'v:0', '-count_frames',
             '-show_entries', 'stream=codec_name,profile,pix_fmt,width,height,nb_read_frames',
             '-of', 'json'], capture_output=True, text=True, timeout=20, check=True)
        stream = json.loads(result.stdout)['streams'][0]
        self.assertEqual(stream['codec_name'], 'h264')
        self.assertEqual((stream['width'], stream['height']), (WIDTH, HEIGHT))
        self.assertGreaterEqual(int(stream['nb_read_frames']), 40, stream)
        return url, stream

    def test_production_x264_is_8bit_420_and_decodes_nonblack_frames(self):
        url, stream = self.probe()
        # pix_fmt 明确区分 8 bit 与 yuv420p10le，且排除 High 4:4:4。
        self.assertEqual(stream['pix_fmt'], 'yuv420p', stream)
        self.assertNotIn('4:4:4', stream['profile'], stream)
        decoded = subprocess.run(
            ['ffmpeg', '-v', 'error', '-nostdin', '-rtsp_transport', 'tcp', '-i', url,
             '-frames:v', '3', '-pix_fmt', 'rgb24', '-f', 'rawvideo', 'pipe:1'],
            capture_output=True, timeout=15, check=True).stdout
        frame_bytes = WIDTH * HEIGHT * 3
        self.assertEqual(len(decoded), frame_bytes * 3)
        for frame in range(3):
            offset = frame * frame_bytes + (100 * WIDTH + 100) * 3
            red, green, blue = decoded[offset:offset + 3]
            self.assertGreater(red, 200, (red, green, blue))
            self.assertLess(green, 70, (red, green, blue))
            self.assertLess(blue, 60, (red, green, blue))

    def test_unrestricted_rgb_pipeline_exposes_444_regression(self):
        _, stream = self.probe(legacy=True)
        self.assertEqual(stream['pix_fmt'], 'yuv444p', stream)
        self.assertIn('4:4:4', stream['profile'], stream)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--worker':
        worker(int(sys.argv[2]), bool(int(sys.argv[3])))
    else:
        unittest.main()
