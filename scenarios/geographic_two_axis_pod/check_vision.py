#!/usr/bin/env python3
"""验收飞机多目标检测、服务端红框推流和自动锁定模式。"""
import argparse
import json
from pathlib import Path
import socket
import time


class Client:
    def __init__(self, host, port):
        self.server = (socket.gethostbyname(host), port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 0))
        self.sock.settimeout(.1)
        self.seq = 0
        self.telemetry = None
        self.next_ping = 0.0

    def send(self, cmd, **extra):
        self.seq += 1
        payload = dict(cmd=cmd, seq=self.seq, **extra)
        self.sock.sendto(json.dumps(payload).encode(), self.server)
        return self.seq, payload

    def wait(self, predicate, timeout, retry=None):
        deadline = time.monotonic() + timeout
        next_retry = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= self.next_ping:
                self.send('ping')
                self.next_ping = now + 1.0
            if retry and time.monotonic() >= next_retry:
                self.sock.sendto(json.dumps(retry).encode(), self.server)
                next_retry = time.monotonic() + .2
            try:
                data, source = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            if source != self.server:
                continue
            message = json.loads(data)
            if message.get('type') == 'telemetry':
                self.telemetry = message
            if predicate(message):
                return message
        raise AssertionError('等待网关状态超时')

    def set_mode(self, mode):
        seq, payload = self.send('set_vision_mode', mode=mode)
        result = self.wait(lambda m: m.get('type') == 'command_result'
                           and m.get('seq') == seq, 5, payload)
        assert result.get('ok'), result

    def close(self, leave_mode='off'):
        try:
            self.set_mode(leave_mode)
            self.send('bye')
        finally:
            self.sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--min-targets', type=int, default=1)
    parser.add_argument('--output', type=Path,
                        default=Path('runs/geographic_two_axis_pod/vision.json'))
    parser.add_argument('--leave-mode', choices=('off', 'detect'), default='off',
                        help='验收结束后保留的视觉模式；保存带框视频截图时使用 detect')
    args = parser.parse_args()
    client = Client(args.host, args.port)
    report = {}
    try:
        client.send('ping')
        hello = client.wait(lambda m: m.get('type') == 'hello', 5)
        assert hello.get('version') == 7 and hello.get('angle_convention') == 'motion_head_joint_right_positive', '两轴网关必须使用协议 7'
        capabilities = set(hello.get('capabilities') or [])
        required = {'vision_mode', 'server_video_overlay', 'auto_lock',
                    'frame_exact_track'}
        assert required <= capabilities, f'网关缺少视觉能力: {required-capabilities}'

        client.set_mode('detect')
        detected = client.wait(
            lambda m: m.get('type') == 'telemetry'
            and (m.get('vision') or {}).get('mode') == 'detect'
            and (m.get('vision') or {}).get('target_count', 0) >= args.min_targets,
            12)
        # 首次模型加载计入最初几帧的墙钟窗口；等待滑动速率进入稳态再验收。
        detected = client.wait(
            lambda m: m.get('type') == 'telemetry'
            and (m.get('vision') or {}).get('target_count', 0) >= args.min_targets
            and float((m.get('vision') or {}).get('input_fps', 0)) > 0
            and float((m.get('vision') or {}).get('output_fps', 0))
            >= float((m.get('vision') or {}).get('input_fps', 0)) * .8,
            8)
        vision = detected['vision']
        assert detected.get('protocol_version') == 7, '两轴插件必须使用协议 7'
        assert not vision.get('error'), vision
        input_fps = float(vision.get('input_fps', 0))
        output_fps = float(vision.get('output_fps', 0))
        assert input_fps > 0 and output_fps >= input_fps * .8, (
            f'视觉处理未及时消费相机帧: {output_fps:.2f}/{input_fps:.2f} fps')
        report['detect'] = dict(target_count=vision['target_count'],
                                inference_ms=vision.get('inference_ms'),
                                input_fps=input_fps, output_fps=output_fps)

        client.set_mode('auto_lock')
        locked = client.wait(
            lambda m: m.get('type') == 'telemetry'
            and (m.get('vision') or {}).get('mode') == 'auto_lock'
            and (m.get('vision') or {}).get('locked_track_id') is not None
            and (m.get('tracking') or {}).get('state') == 'TRACK', 12)
        locked_id = locked['vision']['locked_track_id']
        stable_until = time.monotonic() + 2
        while time.monotonic() < stable_until:
            sample = client.wait(lambda m: m.get('type') == 'telemetry', 1)
            assert sample['vision']['locked_track_id'] == locked_id, '锁定后发生目标切换'
        report['auto_lock'] = dict(track_id=locked_id,
                                   target_count=locked['vision']['target_count'],
                                   tracking_state=locked['tracking']['state'])
        report['result'] = 'passed'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n',
                               encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        client.close(args.leave_mode)


if __name__ == '__main__':
    main()
