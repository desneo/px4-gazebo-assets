#!/usr/bin/env python3
"""中科独立二进制UDP网关、真值初选及单路硬件视频服务。"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import socket
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from common.config import DEFAULT_CONFIG, load_config
from common import protocol
if __package__:
    from .truth_selection import choose_candidate
    from .udp_session import UdpSession
else:
    from truth_selection import choose_candidate
    from udp_session import UdpSession


def finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


class BridgeCore:
    """与Gazebo/socket解耦的控制和测量状态；外部UDP始终只有厂商帧。"""
    def __init__(self, config, publish, diagnostics=lambda _value: None, *, clock=time.monotonic):
        self.config = config
        self.publish = publish
        self.publish_diagnostics = diagnostics
        self.clock = clock
        self.lock = threading.RLock()
        self.mode = 'IDLE'
        self.selection_mode = 'OFF'
        self.tracking_state = 'IDLE'
        self.bound_name = None
        self.bound_id = None
        self.target_valid = False
        self.reason = '未选择目标'
        self.selection_note = ''
        self.latest = None
        self.latest_mono = None
        self.last_sample_time = None
        self.last_acquire = float('-inf')
        self.last_result = None
        self.video_error = ''
        self.initialized = False
        self.generation = 0
        self.counter = 0
        self.instance = uuid.uuid4().hex[:12]
        self.requests = {}
        self.pending_request = None
        self.session = UdpSession(config['network']['peer_lease_s'],
            config['network']['manual_timeout_s'], clock=clock, on_stop=self._timeout_stop)

    def _emit(self, cmd, **values):
        self.counter += 1
        request_id = f'{self.instance}-{self.generation}-{self.counter}'
        self.requests[request_id] = self.generation
        if len(self.requests) > 64:
            del self.requests[next(iter(self.requests))]
        self.publish(dict(cmd=cmd, request_id=request_id, **values))
        return request_id

    def _cancel_selection(self, reason):
        self.generation += 1
        self.selection_mode = 'OFF'
        self.bound_name = self.bound_id = self.pending_request = None
        self.target_valid = False
        self.tracking_state = 'IDLE'
        self.reason = reason
        self.selection_note = ''

    def _timeout_stop(self, reason):
        self._cancel_selection('客户端租约超时' if reason == 'peer_timeout' else '人工速度命令超时')
        self.mode = 'IDLE'
        self._emit('stop')
        self.diagnostics()

    def _fresh(self):
        return (self.latest is not None and self.latest_mono is not None
                and self.clock() - self.latest_mono <= self.config['network']['telemetry_max_age_s'])

    def _command_parameters_valid(self, frame):
        if frame['cmd'] == protocol.CMD_FRAME_ANGLE:
            m = self.config['mechanics']
            if not (m['yaw_min_deg'] <= frame['x'] / 100 <= m['yaw_max_deg']
                    and m['pitch_min_deg'] <= frame['y'] / 100 <= m['pitch_max_deg']):
                raise ValueError('指定框架角超出配置机械行程')
        if frame['cmd'] == protocol.CMD_POINT_TRACK:
            w, h = self.config['optics']['width_px'], self.config['optics']['height_px']
            if not (-w / 2 <= frame['x'] < w / 2 and -h / 2 <= frame['y'] < h / 2):
                raise ValueError('指点坐标超出原始图像范围')

    def receive(self, raw, peer):
        with self.lock:
            try:
                frame = protocol.decode_command(raw)
                self._command_parameters_valid(frame)
            except ValueError as exc:
                self.reason = str(exc)
                self.diagnostics()
                return False
            frame = self.session.receive(raw, peer)
            if frame is None:
                self.reason = self.session.last_error
                self.diagnostics()
                return False
            cmd = frame['cmd']
            if cmd == protocol.CMD_NOP:
                return True
            if cmd == protocol.CMD_AUTO_LOCK and self.selection_mode == 'AUTO':
                # 无序号的自动开启命令须幂等；续租不能抢占健康绑定。
                return True
            self.last_result = {'command': cmd, 'state': '已发送至组件内部控制'}
            self._cancel_selection('人工接管')
            if cmd == protocol.CMD_MANUAL:
                maximum = self.config['mechanics']['max_rate_dps']
                yaw = max(-maximum, min(maximum, frame['x'] / 10))
                pitch = max(-maximum, min(maximum, frame['y'] / 10))
                self.mode = 'MANUAL' if yaw or pitch else 'IDLE'
                self._emit('manual', yaw=yaw, pitch=pitch)
            elif cmd == protocol.CMD_FRAME_ANGLE:
                self.mode = 'HOME'
                self._emit('set_angles', yaw=frame['x'] / 100, pitch=frame['y'] / 100)
            elif cmd == protocol.CMD_HOME:
                self.mode = 'HOME'
                self._emit('home')
            elif cmd == protocol.CMD_STOP_TRACK:
                self.mode = 'IDLE'
                self.reason = '已停止跟踪'
                self._emit('stop')
            elif cmd in (protocol.CMD_POINT_TRACK, protocol.CMD_AUTO_LOCK):
                self._emit('stop')
                self.selection_mode = 'POINT' if cmd == protocol.CMD_POINT_TRACK else 'AUTO'
                self.mode = 'TRACK'
                self.tracking_state = 'AUTO_WAIT' if self.selection_mode == 'AUTO' else 'LOST'
                point = None if self.selection_mode == 'AUTO' else (
                    frame['x'] + self.config['optics']['width_px'] / 2,
                    frame['y'] + self.config['optics']['height_px'] / 2)
                self._acquire(point)
            self.diagnostics()
            return True

    def _acquire(self, point=None):
        self.last_acquire = self.clock()
        if not self._fresh() or self.latest.get('camera_ready') is not True:
            self.reason = '等待新鲜视频与真值候选'
            return
        candidate, fallback = choose_candidate(self.latest, self.config, point)
        if candidate is None:
            self.bound_name = self.bound_id = None
            self.reason = '发现距离内没有画面内候选'
            return
        self.bound_name, self.bound_id = candidate['name'], candidate['entity_id']
        self.selection_note = '已回退选择画面内最近候选' if fallback else ''
        self.target_valid = False
        self.tracking_state = 'INIT'
        self.reason = '已回退选择画面内最近候选' if fallback else '等待组件确认目标绑定'
        self.pending_request = self._emit('track_entity', entity_name=self.bound_name,
            entity_id=self.bound_id, frame_seq=candidate['frame_seq'])

    def on_result(self, result):
        with self.lock:
            request = result.get('request_id')
            if self.requests.get(request) != self.generation:
                return
            self.last_result = dict(result)
            if request == self.pending_request and result.get('ok') is not True:
                self.target_valid = False
                self.tracking_state = 'LOST'
                self.reason = result.get('message') or '组件拒绝目标绑定'
                self.pending_request = None
            self.diagnostics()

    def on_telemetry(self, data):
        """每份新物理状态最多产生一份标准帧，非法机械测量不填默认零。"""
        with self.lock:
            try:
                if (data.get('config_hash') != self.config['config_hash']
                        or data.get('configured') is not True):
                    raise ValueError('插件配置未就绪或性能hash不一致')
                if data.get('mechanical_valid') is not True:
                    raise ValueError('机械测量无效')
                joints = data['joints_deg']
                values = [joints[axis][key] for axis in ('yaw', 'pitch') for key in ('pos', 'vel')]
                stamp = data['ts_mono']
                if not all(finite(v) for v in (*values, stamp)):
                    raise ValueError('关节测量或物理时间不是有限值')
                if self.last_sample_time == stamp:
                    return None
                if self.last_sample_time is not None and stamp < self.last_sample_time:
                    self._cancel_selection('仿真时间重置')
                    self._emit('stop')
                self.last_sample_time = stamp
                self.latest, self.latest_mono = data, self.clock()
                target = data.get('target', {})
                miss = target.get('miss', {})
                valid = (self.selection_mode != 'OFF' and self.bound_name is not None
                    and data.get('mode') == 'TRACK' and data.get('tracking', {}).get('state') == 'TRACK'
                    and target.get('valid') is True and miss.get('valid') is True
                    and target.get('entity_name') == self.bound_name
                    and target.get('entity_id') == self.bound_id)
                offset = miss.get('offset_deg')
                if valid and (not isinstance(offset, (tuple, list)) or len(offset) != 2
                              or not all(finite(v) for v in offset)):
                    raise ValueError('有效跟踪缺少有限角偏差')
                self.target_valid = valid
                if valid:
                    self.tracking_state = 'TRACK'
                    if not self.reason.startswith('已回退'):
                        self.reason = '有效跟踪'
                    self.pending_request = None
                elif self.bound_name is not None:
                    self.tracking_state = data.get('tracking', {}).get('state', 'TEMP_LOST')
                    if self.tracking_state in ('IDLE', 'TRACK'):
                        self.tracking_state = 'TEMP_LOST'
                    self.reason = data.get('tracking', {}).get('message') or '目标观测无效'
                if self.selection_mode == 'AUTO' and not valid:
                    if self.clock() - self.last_acquire >= self.config['tracking']['reacquire_period_s']:
                        self.tracking_state = 'AUTO_WAIT'
                        self._acquire()
                elif self.selection_mode == 'OFF':
                    self.mode = data.get('mode', self.mode)
                camera_ok = data.get('camera_ready') is True and not self.video_error
                fault = bool(data.get('tracking', {}).get('fault'))
                if camera_ok and not fault:
                    self.initialized = True
                self_check = (0x80 if self.initialized else 0) | (0x02 if fault else 0) | (0 if camera_ok else 1)
                fields = dict(status1=4 if valid else 0,
                    status2=8 if data.get('motor_enabled') is True and not fault else 0, status3=0,
                    zoom=self.config['optics']['focal_length_mm'] / self.config['optics']['reference_focal_length_mm'],
                    offset_x=offset[0] if valid else 0.0, offset_y=-offset[1] if valid else 0.0,
                    frame_roll=0.0, frame_pitch=joints['pitch']['pos'], frame_yaw=joints['yaw']['pos'],
                    roll_rate=0.0, pitch_rate=joints['pitch']['vel'], yaw_rate=joints['yaw']['vel'],
                    laser_range=0.0, self_check=self_check)
                raw = protocol.encode_telemetry(fields)
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                self.target_valid = False
                self.reason = f'遥测无效：{exc}'
                self.diagnostics()
                return None
            self.diagnostics()
            return raw

    def poll(self):
        with self.lock:
            self.session.poll()
            if self.latest_mono is not None and not self._fresh() and self.target_valid:
                self.target_valid = False
                self.tracking_state = 'TEMP_LOST'
                self.reason = '组件遥测已过期'
                self._emit('stop')
                self.diagnostics()

    def diagnostics(self):
        data = self.latest or {}
        payload = dict(model_name=self.config['model']['name'], config_hash=self.config['config_hash'],
            mode=self.mode, selection_mode=self.selection_mode, tracking_state=self.tracking_state,
            target_name=self.bound_name, target_valid=self.target_valid,
            reason=(self.selection_note + '；' + self.reason) if self.selection_note else self.reason,
            limits=data.get('limits', {}), singular=data.get('singular', False),
            command_result=self.last_result, target_box_px=data.get('target', {}).get('box_px') if self.target_valid else None,
            config={'optics': {**self.config['optics'], **self.config['derived']},
                    'mechanics': self.config['mechanics']}, diagnostic_mono=self.clock())
        self.publish_diagnostics(payload)
        return payload


class Gateway:
    def __init__(self, config, *, topic_prefix=None, image_topic=None, video=True,
                 world=None, model=None, session=None):
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.msgs10.image_pb2 import Image
        from gz.transport13 import Node
        self.config = config
        self.StringMsg = StringMsg
        self.node = Node()
        self.prefix = topic_prefix or '/' + config['model']['name']
        self.session_name = session
        self.command_pub = self.node.advertise(self.prefix + '/command', StringMsg)
        self.diagnostic_pub = self.node.advertise(self.prefix + '/diagnostics', StringMsg)
        self.core = BridgeCore(config, self._publish, self._diagnostic)
        family = socket.AF_INET6 if ':' in config['network']['udp_bind'] else socket.AF_INET
        self.sock = socket.socket(family, socket.SOCK_DGRAM)
        self.sock.bind((config['network']['udp_bind'], config['network']['udp_port']))
        self.sock.settimeout(.05)
        self.closing = threading.Event()
        self.tx_count = 0
        self.video = None
        self.video_enabled = video
        self.image_seq = 0
        self.source = None
        if video:
            if __package__:
                from .gz_source import GzSource
            else:
                from gz_source import GzSource
            self.source = GzSource(world or config['demo']['world_name'], model or config['model']['name'])
            self.node.subscribe(Image, image_topic or self.prefix + '/camera', self._on_image)
        self.node.subscribe(StringMsg, self.prefix + '/telemetry', self._on_telemetry)
        self.node.subscribe(StringMsg, self.prefix + '/command_result', self._on_result)

    def _publish(self, payload):
        return self.command_pub.publish(self.StringMsg(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))

    def _diagnostic(self, payload):
        self.diagnostic_pub.publish(self.StringMsg(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))

    def _on_telemetry(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
            with self.core.lock:
                raw = self.core.on_telemetry(data)
                peer = self.core.session.telemetry_peer()
                if raw is not None and peer is not None:
                    self.sock.sendto(raw, peer)
                    self.tx_count += 1
        except (ValueError, TypeError, OSError) as exc:
            if not self.closing.is_set():
                print(f'中科遥测处理失败：{exc}', flush=True)

    def _on_result(self, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                self.core.on_result(value)
        except (ValueError, TypeError):
            return

    def _on_image(self, msg):
        if self.closing.is_set():
            return
        try:
            o = self.config['optics']
            if (msg.width, msg.height) != (o['width_px'], o['height_px']) or len(msg.data) != msg.width * msg.height * 3:
                raise ValueError('相机RGB尺寸与当前配置不一致')
            if self.video is None:
                if __package__:
                    from .video import VideoStreamer
                else:
                    from video import VideoStreamer
                v = self.config['video']
                self.video = VideoStreamer(o['width_px'], o['height_px'], fps=o['fps'],
                    rtsp_port=v['rtsp_port'], rtsp_bind=v['rtsp_bind'], mount=v['rtsp_mount'],
                    bitrate_kbps=v['bitrate_kbps'], on_error=self._video_error,
                    on_relay_pid=self._record_relay_pid)
                print(f"中科RTSP已启动：rtsp://{v['rtsp_bind']}:{v['rtsp_port']}{v['rtsp_mount']}", flush=True)
            self.image_seq += 1
            sim = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            offset = self.source.clock_offset if self.source else 0
            self.video.push(self.image_seq & 0xFFFFFF, bytes(msg.data), sim + offset if offset else time.monotonic())
        except Exception as exc:
            self._video_error(str(exc))

    def _record_relay_pid(self, pid):
        if self.session_name:
            from scripts.session import record_process
            record_process(self.session_name, 'rtsp_relay', pid)

    def _video_error(self, message):
        with self.core.lock:
            self.core.video_error = message
            self.core.reason = f'视频硬件链路失败：{message}'
            self.core.diagnostics()
        print(f'中科视频错误：{message}', flush=True)
        self.closing.set()

    def run(self):
        for sig in (signal.SIGTERM, signal.SIGINT):
            signal.signal(sig, lambda *_args: self.closing.set())
        print(f'中科网关启动：UDP {self.sock.getsockname()}，仅16字节控制/32字节遥测', flush=True)
        try:
            while not self.closing.is_set():
                try:
                    raw, peer = self.sock.recvfrom(2048)
                    self.core.receive(raw, peer)
                except socket.timeout:
                    pass
                self.core.poll()
        finally:
            with self.core.lock:
                self.core._timeout_stop('peer_timeout')
            self.closing.set()
            self.sock.close()
            if self.video is not None:
                self.video.close()


def main():
    parser = argparse.ArgumentParser(description='中科DYT独立二进制UDP网关')
    parser.add_argument('--config', default=str(DEFAULT_CONFIG))
    parser.add_argument('--topic-prefix')
    parser.add_argument('--image-topic')
    parser.add_argument('--world')
    parser.add_argument('--model')
    parser.add_argument('--session')
    parser.add_argument('--no-video', action='store_true')
    args = parser.parse_args()
    try:
        Gateway(load_config(args.config), topic_prefix=args.topic_prefix,
                image_topic=args.image_topic, video=not args.no_video,
                world=args.world, model=args.model, session=args.session).run()
    except (ValueError, OSError, ImportError, RuntimeError) as exc:
        print(f'中科网关启动失败：{exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
