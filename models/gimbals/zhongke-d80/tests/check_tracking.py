#!/usr/bin/env python3
"""在已有独占中科会话中，用set_pose和标准UDP核验真值选择；不启动世界。"""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
import multiprocessing
import os
from pathlib import Path
import socket
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common.config import load_config
from common import protocol
from scripts.session import load_session


def rotate(quaternion, vector):
    """归一化w/x/y/z四元数，将相机局部向量旋转到世界。"""
    norm = math.sqrt(sum(x * x for x in quaternion))
    w, x, y, z = (value / norm for value in quaternion)
    vx, vy, vz = vector
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def service_worker(connection, partition, world):
    """单独请求进程不订阅高频图像/遥测，避免绑定层GIL竞争阻塞服务响应。"""
    os.environ['GZ_PARTITION'] = partition
    from gz.transport13 import Node
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.scene_pb2 import Scene
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.boolean_pb2 import Boolean
    node = Node()
    while True:
        request = connection.recv()
        if request['op'] == 'quit':
            break
        try:
            if request['op'] == 'scene':
                success, scene = node.request(f'/world/{world}/scene/info', Empty(), Empty, Scene, 3000)
                if not success:
                    raise RuntimeError('读取实际场景原姿态超时')
                result = {}
                for model in scene.model:
                    pose = model.pose
                    result[model.name] = dict(position=[pose.position.x, pose.position.y, pose.position.z],
                        quaternion=[pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
            else:
                pose = Pose()
                pose.name = request['name']
                pose.position.x, pose.position.y, pose.position.z = request['position']
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = request['quaternion']
                success, response = node.request(f'/world/{world}/set_pose', pose, Pose, Boolean, 2000)
                if not success or not response.data:
                    raise RuntimeError('目标set_pose没有成功响应：' + pose.name)
                result = True
            connection.send(dict(ok=True, result=result))
        except Exception as exc:
            connection.send(dict(ok=False, error=str(exc)))
    connection.close()


class TrackingProbe:
    def __init__(self, session, output):
        self.session, self.output = session, output
        output.mkdir(parents=True, exist_ok=True)
        self.config = load_config(session['config_path'])
        os.environ['GZ_PARTITION'] = session['gz_partition']
        from gz.transport13 import Node
        from gz.msgs10.stringmsg_pb2 import StringMsg
        self.node = Node()
        self.lock = threading.RLock()
        self.latest = self.diagnostic = self.binary = None
        self.latest_at = self.diagnostic_at = self.binary_at = 0.0
        self.rows, self.cases, self.service_calls = [], [], []
        self.original_poses = {}
        self.phase = '连接'
        self.errors = []
        self.names = self.config['tracking']['target_names'][:3]
        if len(self.names) < 3:
            raise ValueError('本验收夹具需要配置至少三个已存在的注册目标')
        self.maximum = self.config['tracking']['discovery_range_m']
        if self.maximum <= 2:
            raise ValueError('299/300/301式边界夹具要求发现距离大于2m')
        self.world = session.get('world', self.config['demo']['world_name'])
        context = multiprocessing.get_context('spawn')
        self.service_connection, child_connection = context.Pipe()
        self.service_process = context.Process(target=service_worker,
            args=(child_connection, session['gz_partition'], self.world), daemon=True)
        self.service_process.start()
        child_connection.close()
        self.prefix = session.get('topic_prefix', '/' + self.config['model']['name'])
        self.node.subscribe(StringMsg, self.prefix + '/telemetry', self._on_telemetry)
        self.node.subscribe(StringMsg, session.get('diagnostics_topic', self.prefix + '/diagnostics'), self._on_diagnostic)
        self.destination = (session['udp_host'], int(session['udp_port']))
        family = socket.AF_INET6 if ':' in self.destination[0] else socket.AF_INET
        self.sock = socket.socket(family, socket.SOCK_DGRAM)
        self.sock.bind(('::' if family == socket.AF_INET6 else '0.0.0.0', 0))
        self.sock.connect(self.destination)
        self.sock.settimeout(.1)
        self.closed = threading.Event()
        self.threads = (threading.Thread(target=self._heartbeat, daemon=True),
                        threading.Thread(target=self._receive, daemon=True))
        for thread in self.threads:
            thread.start()

    def _heartbeat(self):
        while not self.closed.is_set():
            try:
                self.send(protocol.CMD_NOP)
            except OSError:
                pass
            self.closed.wait(.15)

    def _receive(self):
        while not self.closed.is_set():
            try:
                fields = protocol.decode_telemetry(self.sock.recv(2048))
            except socket.timeout:
                continue
            except (OSError, ValueError):
                continue
            with self.lock:
                self.binary, self.binary_at = fields, time.monotonic()
                self.rows.append(dict(kind='binary', wall_time=self.binary_at, phase=self.phase, fields=fields))

    def _on_telemetry(self, message):
        try:
            value = json.loads(message.data)
            if value.get('config_hash') != self.config['config_hash']:
                return
            with self.lock:
                self.latest, self.latest_at = value, time.monotonic()
                self.rows.append(dict(kind='plugin', wall_time=self.latest_at, phase=self.phase,
                    sim_time=value.get('ts_mono'), mode=value.get('mode'), joints_deg=value.get('joints_deg'),
                    target=value.get('target'), candidates=value.get('truth_candidates'),
                    camera=value.get('camera')))
        except (ValueError, TypeError):
            return

    def _on_diagnostic(self, message):
        try:
            value = json.loads(message.data)
            if (value.get('config_hash') != self.config['config_hash']
                    or value.get('model_name') != self.config['model']['name']):
                return
            with self.lock:
                self.diagnostic, self.diagnostic_at = value, time.monotonic()
                self.rows.append(dict(kind='diagnostic', wall_time=self.diagnostic_at, phase=self.phase, fields=value))
        except (ValueError, TypeError):
            return

    def snapshot(self):
        with self.lock:
            return deepcopy(dict(plugin=self.latest, diagnostic=self.diagnostic, binary=self.binary,
                plugin_at=self.latest_at, diagnostic_at=self.diagnostic_at, binary_at=self.binary_at))

    def wait(self, predicate, label, timeout=6, after=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = self.snapshot()
            now = time.monotonic()
            fresh = all(value[key] is not None for key in ('plugin', 'diagnostic', 'binary'))
            fresh = fresh and all(now - value[key + '_at'] < self.config['network']['telemetry_max_age_s']
                                  for key in ('plugin', 'diagnostic', 'binary'))
            if after is not None:
                fresh = fresh and all(value[key + '_at'] > after for key in ('plugin', 'diagnostic', 'binary'))
            if fresh and predicate(value):
                return value
            time.sleep(.01)
        last = self.snapshot()
        brief = dict(diagnostic=last['diagnostic'], binary=last['binary'],
                     candidates=(last['plugin'] or {}).get('truth_candidates'))
        raise AssertionError(f'{label}超时：{brief}')

    def send(self, cmd, x=0, y=0):
        self.sock.send(protocol.encode_command(cmd, x, y))

    def set_pose(self, name, position=None, saved=None):
        pose = deepcopy(saved) if saved is not None else dict(position=list(map(float, position)),
                                                            quaternion=[1.0, 0.0, 0.0, 0.0])
        self.request_service(dict(op='pose', name=name, **pose))
        self.service_calls.append(dict(phase=self.phase, name=name, **pose, wall_time=time.monotonic()))

    def request_service(self, request):
        self.service_connection.send(request)
        if not self.service_connection.poll(5):
            raise AssertionError('Gazebo请求进程无响应：' + request['op'])
        response = self.service_connection.recv()
        if not response['ok']:
            raise AssertionError(response['error'])
        return response['result']

    def save_original_poses(self):
        poses = self.request_service(dict(op='scene'))
        for name in self.names:
            if name in poses:
                self.original_poses[name] = poses[name]
        if set(self.original_poses) != set(self.names):
            raise AssertionError(f'场景中缺少已配置目标：{set(self.names) - set(self.original_poses)}')

    def camera(self):
        value = self.snapshot()['plugin'] or {}
        candidates = value.get('truth_candidates', [])
        center = next((item.get('camera_optical_center') for item in candidates
                       if isinstance(item.get('camera_optical_center'), list)), None)
        quaternion = value.get('camera', {}).get('quat_world')
        if center is None or not isinstance(quaternion, list) or len(quaternion) != 4:
            raise AssertionError('新鲜内部遥测缺少实际光学中心/世界旋转，不能以模型原点替代')
        return center, quaternion

    def position(self, distance, dx=0, dy=0):
        center, quaternion = self.camera()
        optics = self.config['derived']
        ray = (1, -dx / optics['fx'], -dy / optics['fy'])
        norm = math.sqrt(sum(x * x for x in ray))
        direction = rotate(quaternion, [x / norm for x in ray])
        return [origin + distance * axis for origin, axis in zip(center, direction)]

    def move(self, name, distance, dx=0, dy=0):
        position = self.position(distance, dx, dy)
        started = time.monotonic()
        self.set_pose(name, position)
        def placed(value):
            candidate = self.candidate(value, name)
            return candidate is not None and max(abs(a - b) for a, b in
                zip(candidate.get('entity_position', []), position)) < .0001
        self.wait(placed, f'{name}新目标姿态同步', after=started)
        return self.wait_candidate(name)

    @staticmethod
    def candidate(snapshot, name):
        return next((item for item in (snapshot['plugin'] or {}).get('truth_candidates', [])
                     if item.get('name') == name), None)

    def wait_candidate(self, name, *, visible=None):
        def ready(value):
            candidate = self.candidate(value, name)
            return candidate is not None and (visible is None or candidate.get('valid') is visible)
        value = self.wait(ready, name + '候选同步')
        return self.candidate(value, name)

    def home(self):
        self.send(protocol.CMD_STOP_TRACK)
        self.send(protocol.CMD_MANUAL)
        before = time.monotonic()
        self.send(protocol.CMD_HOME)
        m = self.config['mechanics']
        timeout = max(m['yaw_max_deg'] - m['yaw_min_deg'], m['pitch_max_deg'] - m['pitch_min_deg']) / m['max_rate_dps'] + 8
        return self.wait(lambda value: value['diagnostic']['selection_mode'] == 'OFF'
            and all(abs(value['plugin']['joints_deg'][axis]['pos']) < .08
                    and abs(value['plugin']['joints_deg'][axis]['vel']) < .05 for axis in ('yaw', 'pitch')),
            '人工回零', timeout, after=before)

    def prepare(self, phase):
        self.phase = phase
        print('真值验收：' + phase, flush=True)
        self.home()
        for name in self.names:
            self.move(name, -50)
        self.wait(lambda value: not any(item.get('valid') for item in value['plugin'].get('truth_candidates', [])),
                  '隐藏其他注册候选')

    def point(self, candidate=None, pixels=None):
        if pixels is None:
            pixels = candidate['center_px']
        x = protocol.quantize(pixels[0] - self.config['optics']['width_px'] / 2, 1, -32768, 32767)
        y = protocol.quantize(pixels[1] - self.config['optics']['height_px'] / 2, 1, -32768, 32767)
        self.send(protocol.CMD_POINT_TRACK, x, y)

    def locked(self, name, after=None):
        return self.wait(lambda value: value['diagnostic'].get('target_name') == name
            and value['diagnostic'].get('target_valid') is True
            and bool(value['binary']['status1'] & 4)
            and value['plugin']['target'].get('entity_name') == name,
            '锁定' + name, after=after)

    def hold(self, seconds, predicate, label):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            value = self.wait(lambda _value: True, '持续新鲜数据')
            if not predicate(value):
                raise AssertionError(label + '：' + str(value['diagnostic']))
            time.sleep(.02)
        return value

    def record(self, label, **extra):
        value = self.snapshot()
        self.cases.append(dict(case=label, result='passed', diagnostic=value['diagnostic'],
            binary=value['binary'], candidates=value['plugin'].get('truth_candidates'), **extra))

    def tracking_motion(self):
        """仅用实际TRACK有效观测期间的关节测量及连续物理时间差分验收。"""
        mechanical = self.config['mechanics']
        limit = mechanical['max_rate_dps']
        angle_epsilon, rate_epsilon = 1e-5, 1e-4
        rows = [row for row in self.rows if row['kind'] == 'plugin']
        def active(row):
            target = row.get('target') or {}
            return row.get('mode') == 'TRACK' and target.get('valid') is True and target.get('miss', {}).get('valid') is True
        tracked = [row for row in rows if active(row)]
        if len(tracked) < 20:
            raise AssertionError('有效TRACK物理采样不足，不能证明跟踪运动约束')
        peak, difference_peak = {}, {}
        for axis in ('yaw', 'pitch'):
            velocities = []
            for row in tracked:
                joint = row['joints_deg'][axis]
                position, velocity = joint['pos'], joint['vel']
                if not math.isfinite(position) or not math.isfinite(velocity):
                    raise AssertionError('TRACK出现非有限物理测量')
                if not mechanical[axis + '_min_deg'] - angle_epsilon <= position <= mechanical[axis + '_max_deg'] + angle_epsilon:
                    raise AssertionError(f'TRACK {axis}位置超硬限位：{position}')
                if abs(velocity) > limit + rate_epsilon:
                    raise AssertionError(f'TRACK {axis}实际速度超限：{velocity}>{limit}')
                velocities.append(abs(velocity))
            peak[axis] = max(velocities)
            differences = []
            for before, after in zip(rows, rows[1:]):
                if (not active(before) or not active(after) or before['phase'] != after['phase']
                        or before['target'].get('entity_id') != after['target'].get('entity_id')):
                    continue
                elapsed = after['sim_time'] - before['sim_time']
                if not 1e-6 < elapsed < 1:
                    continue
                velocity = abs((after['joints_deg'][axis]['pos'] - before['joints_deg'][axis]['pos']) / elapsed)
                if velocity > limit + rate_epsilon:
                    raise AssertionError(f'TRACK {axis}独立位置差分超限：{velocity}>{limit}')
                differences.append(velocity)
            if not differences:
                raise AssertionError('缺少连续TRACK物理时间样本用于独立位置差分')
            difference_peak[axis] = max(differences)
        return dict(sample_count=len(tracked), configured_limit_dps=limit,
            mechanical_peak_dps=peak, position_difference_peak_dps=difference_peak,
            reached_limit={axis: peak[axis] >= .9 * limit for axis in peak},
            measurement_tolerance=dict(angle_deg=angle_epsilon, rate_dps=rate_epsilon),
            source='有效TRACK的原始joints_deg及物理ts_mono；不取命令值、UDP量化值或GUI')

    def run(self):
        self.wait(lambda value: value['plugin'].get('camera_ready') is True,
                  '本组件视频、真实二进制与只读诊断', 12)
        self.save_original_poses()
        a, b, c = self.names
        width, height = self.config['optics']['width_px'], self.config['optics']['height_px']

        self.prepare('POINT按点击最近')
        self.move(a, 100, -width * .15)
        self.move(b, 100, width * .15)
        self.move(c, 100, width * .25)
        clicked = self.wait_candidate(b, visible=True)
        started = time.monotonic()
        self.point(clicked)
        self.locked(b, after=started)
        self.record('双击按点击最近', clicked_candidate=clicked)

        self.prepare('POINT远离候选仍回退')
        chosen = self.move(a, 100, width * .15)
        pixel = (10, height / 2)
        if math.dist(chosen['center_px'], pixel) <= self.config['tracking']['point_fallback_radius_px']:
            raise AssertionError('回退夹具没有超出提示半径')
        started = time.monotonic()
        self.point(pixels=pixel)
        self.locked(a, after=started)
        self.wait(lambda value: '回退' in value['diagnostic'].get('reason', ''), '回退结果提示')
        self.record('双击画面内最近回退', click_px=pixel)
        self.move(a, -50)
        self.move(b, 100)
        self.wait(lambda value: not value['diagnostic'].get('target_valid') and not value['binary']['status1'] & 4,
                  'POINT失锁清有效状态')
        self.hold(.7, lambda value: not value['diagnostic'].get('target_valid')
                  and value['diagnostic'].get('target_name') != b, 'POINT失锁后不应改锁其他实体')
        self.record('POINT失锁不自动改锁')

        self.prepare('AUTO中心最近与身份保持')
        self.move(a, 100, width * .25)
        self.move(b, 100, width * .025)
        self.move(c, 100, -width * .25)
        self.send(protocol.CMD_AUTO_LOCK)
        self.locked(b)
        # 目标B短暂偏离中心，A位于中心；两者均在视场内。
        self.move(a, 100)
        self.move(b, 100, width * .35)
        closer = self.wait(lambda value: self.candidate(value, a).get('valid')
            and math.dist(self.candidate(value, a)['center_px'], (width / 2, height / 2)) + 5
            < math.dist(self.candidate(value, b)['center_px'], (width / 2, height / 2)),
            '其他候选更靠近画面中心', 3)
        self.hold(.5, lambda value: value['diagnostic'].get('target_name') == b,
                  'AUTO正常锁定被其他候选抢占')
        self.locked(b)
        self.record('AUTO中心最近且不抢占', closer_snapshot=closer['plugin']['truth_candidates'])

        for distance in (self.maximum - 1, self.maximum, self.maximum + 1):
            self.prepare(f'发现距离{distance:g}m')
            candidate = self.move(a, distance)
            if abs(candidate['range_m'] - distance) > .0001:
                raise AssertionError('目标放置误差超过0.1mm，不能证明发现距离边界')
            started = time.monotonic()
            self.point(candidate)
            if distance <= self.maximum:
                self.locked(a, after=started)
                self.record(f'{distance:g}m允许POINT发现', requested_range_m=distance)
                self.home()
                self.send(protocol.CMD_AUTO_LOCK)
                self.locked(a)
                self.record(f'{distance:g}m允许AUTO发现', requested_range_m=distance)
            else:
                self.hold(.5, lambda value: not value['diagnostic'].get('target_valid')
                          and not value['binary']['status1'] & 4, '超距POINT不得锁定')
                self.send(protocol.CMD_AUTO_LOCK)
                self.wait(lambda value: value['diagnostic']['tracking_state'] == 'AUTO_WAIT', '超距AUTO等待')
                self.hold(.5, lambda value: not value['diagnostic'].get('target_valid'), '超距AUTO不得锁定')
                self.record(f'{distance:g}m禁止POINT/AUTO发现', requested_range_m=distance)

        self.prepare('斜向距离不能以depth代替range')
        dx = width * .425
        ratio = math.sqrt(1 + (dx / self.config['derived']['fx']) ** 2)
        distance = self.maximum * (1 + (ratio - 1) / 2)
        candidate = self.move(a, distance, dx)
        if not candidate['valid'] or not candidate['depth_m'] < self.maximum < candidate['range_m']:
            raise AssertionError('斜向夹具必须满足在画面内且depth<上限<range')
        self.send(protocol.CMD_AUTO_LOCK)
        self.wait(lambda value: value['diagnostic']['tracking_state'] == 'AUTO_WAIT', '斜向超距等待')
        self.hold(.6, lambda value: not value['diagnostic'].get('target_valid'), '斜向超距不得发现')
        self.record('发现距离使用三维range', diagonal_candidate=candidate)

        self.prepare('超距持续跟踪与失锁重选边界')
        candidate = self.move(a, self.maximum - 1)
        self.send(protocol.CMD_AUTO_LOCK)
        self.locked(a)
        candidate = self.move(a, self.maximum + 1)
        if not candidate['valid'] or candidate['range_m'] <= self.maximum:
            raise AssertionError('持续跟踪夹具没有移动到画面内超距位置')
        self.hold(.8, lambda value: value['diagnostic'].get('target_name') == a
            and value['diagnostic'].get('target_valid') and value['binary']['status1'] & 4,
            '已锁实体超距后必须继续同实体跟踪')
        self.record('已锁实体超距继续跟踪')
        self.move(a, -50)
        self.wait(lambda value: not value['diagnostic'].get('target_valid')
            and not value['binary']['status1'] & 4
            and value['binary']['offset_x'] == value['binary']['offset_y'] == 0, '失锁清旧偏差')
        self.move(a, self.maximum + 1)
        self.hold(.7, lambda value: not value['diagnostic'].get('target_valid'), '失锁后不能超距重新绑定')
        self.record('失锁后超距不重选')
        self.move(a, self.maximum - 1)
        self.locked(a)
        self.record('AUTO重新出现合格候选后恢复')

        for command, label in ((protocol.CMD_MANUAL, 'MANUAL'), (protocol.CMD_HOME, 'HOME'),
                               (protocol.CMD_STOP_TRACK, 'STOP')):
            self.prepare(label + '人工接管')
            self.move(a, 100, width * .15)
            self.send(protocol.CMD_AUTO_LOCK)
            self.locked(a)
            started = time.monotonic()
            self.send(command, 50 if command == protocol.CMD_MANUAL else 0)
            self.wait(lambda value: value['diagnostic']['selection_mode'] == 'OFF'
                      and not value['diagnostic']['target_valid'] and not value['binary']['status1'] & 4,
                      label + '撤销跟踪', after=started)
            if command == protocol.CMD_MANUAL:
                self.send(protocol.CMD_MANUAL)
            self.hold(.7, lambda value: value['diagnostic']['selection_mode'] == 'OFF'
                and not value['diagnostic']['target_valid'], label + '后旧自动任务不能复活')
            self.record(label + '撤销旧会话与自动重选')
        self.prepare('TRACK实际限速边缘夹具')
        # 合法近垂直姿态的雅可比放大横向像差，实际TRACK可触及限速。
        # 不增加增益、不伪造目标偏差，也不要求任意窄视场配置必然触顶。
        pitch = max(self.config['mechanics']['pitch_min_deg'] + 5, -80)
        pitch = min(pitch, self.config['mechanics']['pitch_max_deg'])
        started = time.monotonic()
        self.send(protocol.CMD_FRAME_ANGLE, 0, protocol.quantize(pitch, '.01', -32768, 32767))
        self.wait(lambda value: abs(value['plugin']['joints_deg']['pitch']['pos'] - pitch) < .08
                  and abs(value['plugin']['joints_deg']['pitch']['vel']) < .05,
                  '合法近垂直框架角', timeout=abs(pitch) / self.config['mechanics']['max_rate_dps'] + 10,
                  after=started)
        candidate = self.move(a, 100, width * .425)
        if not candidate['valid']:
            raise AssertionError('边缘TRACK夹具不在实际画面中')
        self.point(candidate)
        self.locked(a)
        self.hold(.8, lambda value: value['diagnostic'].get('target_name') == a
                  and value['diagnostic'].get('target_valid'), '边缘目标应保持实际有效TRACK')
        self.record('合法边缘候选的真实TRACK运动', pitch_setup_deg=pitch)
        motion = self.tracking_motion()
        self.home()
        return dict(result='passed', session=self.session['session_path'], config_hash=self.config['config_hash'],
            discovery_range_m=self.maximum, placement_tolerance_m=.0001,
            tracking_motion=motion,
            cases=self.cases, service_calls=self.service_calls,
            binary_samples=sum(row['kind'] == 'binary' for row in self.rows),
            evidence='实际Gazebo set_pose、插件真值快照、只读诊断和接收的32字节UDP；未启动世界')

    def close(self):
        self.phase = '恢复目标原姿态'
        try:
            try:
                self.send(protocol.CMD_STOP_TRACK)
                self.send(protocol.CMD_MANUAL)
            except OSError as exc:
                self.errors.append('停止命令发送失败：' + str(exc))
            for name, pose in self.original_poses.items():
                try:
                    self.set_pose(name, saved=pose)
                except Exception as exc:
                    self.errors.append(str(exc))
            if self.original_poses:
                try:
                    self.home()
                except Exception as exc:
                    self.errors.append('恢复后回零失败：' + str(exc))
        finally:
            self.closed.set()
            for thread in self.threads:
                thread.join(timeout=1)
            self.sock.close()
            self.service_connection.send(dict(op='quit'))
            self.service_process.join(timeout=2)
            if self.service_process.is_alive():
                self.service_process.terminate()
                self.service_process.join(timeout=2)
            self.service_connection.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', default='validation')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    session = load_session(args.session)
    output = args.output or Path(session['session_path']).parent / 'tracking'
    probe = TrackingProbe(session, output)
    try:
        result = probe.run()
    except Exception as exc:
        result = dict(result='failed', error=str(exc), cases=probe.cases, service_calls=probe.service_calls)
        print('中科真值验收失败：' + str(exc), file=sys.stderr)
    finally:
        probe.close()
    if probe.errors:
        result.update(result='failed', cleanup_errors=probe.errors)
    with (output / 'samples.jsonl').open('w') as stream:
        for row in probe.rows:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
    (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    if result['result'] == 'passed':
        print(f'中科真实真值验收通过：{len(probe.cases)}项', flush=True)
    return 0 if result['result'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
