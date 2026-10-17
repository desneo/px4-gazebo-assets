#!/usr/bin/env python3
"""真实多进程编码、同端口双路拉流、路径冲突与租约恢复验证；需要 GStreamer。"""
import ast
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import unittest

SERVICE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVICE))
from rtsp_relay import aircraft_mount, request, ensure_relay


def worker(port, aircraft_id):
    import video
    # 独立进程使用生产 NVENC 管线编码红/蓝图像，验证路径与图像来源没有串流。
    stream = video.VideoStreamer(640, 480, rtsp_port=port,
                                 mount=aircraft_mount('/two_axis_pod', aircraft_id))
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    pixel = bytes((255, 0, 0) if aircraft_id == 1 else (0, 0, 255))
    data = pixel * (640 * 480)
    try:
        seq = 0
        while running:
            stream.push(seq, data, time.monotonic())
            seq += 1
            time.sleep(1 / 30)
    finally:
        stream.close()


class MultiRtspTests(unittest.TestCase):
    def setUp(self):
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            self.port = probe.getsockname()[1]
        self.processes = []
        ensure_relay(self.port)

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=10)

    def start_worker(self, aircraft_id):
        process = subprocess.Popen([sys.executable, __file__, '--worker', str(self.port),
                                    str(aircraft_id)], stdout=subprocess.DEVNULL)
        self.processes.append(process)
        time.sleep(1)
        self.assertIsNone(process.poll(), '视频编码进程启动失败')
        return process

    def decoder(self, aircraft_id):
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst
        Gst.init(None)
        pipeline = Gst.parse_launch(
            f'rtspsrc location=rtsp://127.0.0.1:{self.port}/two_axis_pod_{aircraft_id} '
            'protocols=tcp latency=50 ! rtph264depay ! h264parse ! nvh264dec '
            '! videoconvert ! video/x-raw,format=RGB ! appsink name=frames '
            'sync=false max-buffers=1 drop=true')
        pipeline.set_state(Gst.State.PLAYING)
        self.addCleanup(pipeline.set_state, Gst.State.NULL)
        return pipeline.get_by_name('frames')

    def assert_color(self, sink, channel):
        from gi.repository import Gst
        time.sleep(.2)
        sample = sink.emit('try-pull-sample', 8 * Gst.SECOND)
        self.assertIsNotNone(sample, 'RTSP 解码超时')
        buffer = sample.get_buffer()
        pixel = buffer.extract_dup((100 * 640 + 100) * 3, 3)
        self.assertGreater(pixel[channel], 200, pixel)
        self.assertLess(pixel[2 - channel], 40, pixel)

    def test_two_processes_same_port_no_crosstalk_and_independent_stop(self):
        red = self.start_worker(1)
        self.start_worker(2)
        red_sink, blue_sink = self.decoder(1), self.decoder(2)
        self.assert_color(red_sink, 0)
        self.assert_color(blue_sink, 2)
        with self.assertRaisesRegex(RuntimeError, '已注册'):
            request(self.port, dict(op='register', mount='/two_axis_pod_2',
                                    udp_port=40000, token='duplicate'))
        red.terminate()
        red.wait(timeout=10)
        self.assert_color(blue_sink, 2)
        # 正常退出即时释放路径，另一架继续发送。
        self.start_worker(1)
        self.assert_color(self.decoder(1), 0)
        self.assert_color(blue_sink, 2)

    def test_relay_restart_recovers_existing_sender(self):
        self.start_worker(1)
        self.assert_color(self.decoder(1), 0)
        pid = request(self.port, dict(op='ping'))['pid']
        os.kill(pid, signal.SIGKILL)
        deadline = time.monotonic() + 12
        recovered = False
        while time.monotonic() < deadline:
            try:
                state = request(self.port, dict(op='ping'))
                if state['pid'] != pid and '/two_axis_pod_1' in state['paths']:
                    recovered = True
                    break
            except (OSError, ValueError, RuntimeError):
                pass
            time.sleep(.2)
        self.assertTrue(recovered, '中继重启后网关未重新注册')
        self.assert_color(self.decoder(1), 0)

    def test_expiry_and_foreign_port_rejected(self):
        request(self.port, dict(op='register', mount='/two_axis_pod_3', udp_port=40000, token='old'))
        with self.assertRaisesRegex(RuntimeError, 'RTP 端口'):
            request(self.port, dict(op='register', mount='/two_axis_pod_4', udp_port=40000, token='other'))
        time.sleep(11.5)
        request(self.port, dict(op='register', mount='/two_axis_pod_3', udp_port=40000, token='new'))
        with self.assertRaisesRegex(RuntimeError, '已注册'):
            request(self.port, dict(op='remove', mount='/two_axis_pod_3', token='old'))
        request(self.port, dict(op='remove', mount='/two_axis_pod_3', token='new'))


class PathTests(unittest.TestCase):
    def test_suffix_and_station_match(self):
        path = SERVICE.parents[1] / 'ground_station/ground_station.py'
        tree = ast.parse(path.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'rtsp_url')
        namespace = {}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
        for aircraft_id in (0, 1, 23):
            mount = aircraft_mount('/two_axis_pod', aircraft_id)
            self.assertEqual(mount, f'/two_axis_pod_{aircraft_id}')
            url = namespace['rtsp_url'](dict(gimbal_host='127.0.0.1', rtsp_port=8555,
                                             aircraft_id=aircraft_id))
            self.assertEqual(url, f'rtsp://127.0.0.1:8555{mount}')
        self.assertEqual(namespace['rtsp_url'](dict(gimbal_host='host', rtsp_port=8555,
                                                   rtsp_mount='/custom/', aircraft_id=3)),
                         'rtsp://host:8555/custom_3')
        for bad in (-1, True, '1'):
            with self.assertRaises(ValueError):
                aircraft_mount('/two_axis_pod', bad)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--worker':
        worker(int(sys.argv[2]), int(sys.argv[3]))
    else:
        unittest.main()
