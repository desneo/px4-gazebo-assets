#!/usr/bin/env python3
"""验收 PX4 往返飞行中的真实图像跟踪，目标真值只初始化一次并独立评分。

默认只观察人工框选；--initialize 允许一次机械角对准和真实视频框选。
初始化后的控制完全由吊舱 CSRT 与载机姿态补偿处理，本脚本不发布目标真值。
"""

import argparse
from collections import Counter, deque
import itertools
import json
import math
import os
from pathlib import Path
import socket
import threading
import time

from target_fixture import TARGET_POSITION, TARGET_SIZE

ROOT = Path(__file__).resolve().parents[2]
WORLD = 'geographic_rocket_two_axis_pod'
ENTITY = 'rocket_two_axis_pod'


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def length(vector):
    return math.sqrt(dot(vector, vector))


def rotation(quaternion):
    """wxyz 四元数到旋转矩阵。"""
    norm = length(quaternion)
    if norm < 1e-8:
        raise ValueError('零长度姿态四元数')
    w, x, y, z = (value / norm for value in quaternion)
    return ((1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
            (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
            (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)))


def transpose(matrix):
    return tuple(zip(*matrix))


def mv(matrix, vector):
    return tuple(dot(row, vector) for row in matrix)


def mm(a, b):
    return tuple(tuple(dot(row, column) for column in transpose(b)) for row in a)


def joints_rotation(yaw, pitch):
    y, p = math.radians(yaw), math.radians(pitch)
    cy, sy, cp, sp = math.cos(y), math.sin(y), math.cos(p), math.sin(p)
    return ((cy*cp, -sy, cy*sp), (sy*cp, cy, sy*sp), (-sp, 0., cp))


def angle_deg(a, b):
    return math.degrees(math.acos(max(-1., min(1., dot(a, b) / (length(a) * length(b))))))


def percentile(values, fraction=.95):
    values = sorted(value for value in values if value is not None and math.isfinite(value))
    return values[min(len(values)-1, math.ceil(len(values) * fraction)-1)] if values else None


def wrap_deg(value):
    return (value + 180) % 360 - 180


def quaternion_interpolate(a, b, fraction):
    """球面插值，处理 q 与 -q 表示同一旋转的情况。"""
    a = tuple(value / length(a) for value in a)
    b = tuple(value / length(b) for value in b)
    cosine = dot(a, b)
    if cosine < 0:
        b, cosine = tuple(-value for value in b), -cosine
    if cosine > .9995:
        value = tuple((1-fraction)*x + fraction*y for x, y in zip(a, b))
        return tuple(component/length(value) for component in value)
    theta = math.acos(max(-1., min(1., cosine)))
    scale = math.sin(theta)
    return tuple((math.sin((1-fraction)*theta)*x + math.sin(fraction*theta)*y)/scale
                 for x, y in zip(a, b))


def interpolate_pose(rows, stamp, max_delta=.035):
    """只在接近目标时刻的实际位姿之间插值，不外推运动。"""
    return _interpolate(rows, stamp, 'stamp', max_delta, lambda a, b, f: dict(
        stamp=stamp, position=tuple(x+(y-x)*f for x, y in zip(a['position'], b['position'])),
        quaternion=quaternion_interpolate(a['quaternion'], b['quaternion'], f)))


def interpolate_joints(rows, stamp, max_delta=.035):
    return _interpolate(rows, stamp, 'ts_mono', max_delta, lambda a, b, f: dict(
        ts_mono=stamp, joints_deg={axis: dict(pos=a['joints_deg'][axis]['pos']+
            (b['joints_deg'][axis]['pos']-a['joints_deg'][axis]['pos'])*f)
            for axis in ('yaw', 'pitch')}))


def _interpolate(rows, stamp, key, max_delta, mix):
    if not rows:
        return None
    before = next((row for row in reversed(rows) if row[key] <= stamp), None)
    after = next((row for row in rows if row[key] >= stamp), None)
    nearest = min(rows, key=lambda row: abs(row[key]-stamp))
    if abs(nearest[key]-stamp) > max_delta:
        return None
    if before is None or after is None or after[key] == before[key]:
        return nearest
    if stamp-before[key] > max_delta or after[key]-stamp > max_delta:
        return None
    return mix(before, after, (stamp-before[key])/(after[key]-before[key]))


def camera_pose(pose, telemetry):
    """由独立 Gazebo 本体位姿与实际关节角重建光学系，不采用目标反馈。"""
    body = rotation(pose['quaternion'])
    yaw = telemetry['joints_deg']['yaw']['pos']
    pitch = telemetry['joints_deg']['pitch']['pos']
    joint = joints_rotation(yaw, pitch)
    # sensor RPY=(pi,-pi/2,0)：+X视线、+Y图像左、+Z图像上。
    sensor = ((0., 0., 1.), (0., -1., 0.), (1., 0., 0.))
    optical = mm(mm(body, joint), sensor)
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    center = add((0., 0., .134), (.015*cy, .015*sy, .056))
    origin = add(pose['position'], mv(body, add(center, mv(joint, (0., 0., .026)))))
    return origin, optical


def reachable_angles(pose, target):
    body = rotation(pose['quaternion'])
    vector = mv(transpose(body), sub(target, pose['position']))
    pitch = math.degrees(math.atan2(math.hypot(vector[0], vector[1]), vector[2]))
    yaw = math.degrees(math.atan2(vector[1], vector[0]))
    candidates = [(yaw, pitch), (wrap_deg(yaw + 180), -pitch)]
    for y, p in candidates:
        if -90 <= y <= 90 and -45 <= p <= 90:
            return dict(reachable=True, yaw=y, pitch=p, singular=abs(p) < 3)
    return dict(reachable=False, yaw=yaw, pitch=pitch, singular=False)


def stable_cruise(poses, altitude, cruise_speed):
    """要求完整三仿真秒的巡航，排除爬升接近高度或原地悬停。"""
    if len(poses) < 3:
        return False
    recent = [p for p in poses if poses[-1]['stamp']-p['stamp'] <= 3.2]
    if recent[-1]['stamp']-recent[0]['stamp'] < 3:
        return False
    zs = [p['position'][2] for p in recent]
    duration = recent[-1]['stamp']-recent[0]['stamp']
    displacement = sub(recent[-1]['position'], recent[0]['position'])
    horizontal_speed = math.hypot(*displacement[:2])/duration
    vertical_speed = abs(displacement[2])/duration
    return (max(abs(z-altitude) for z in zs) <= 3 and max(zs)-min(zs) <= 2
            and vertical_speed <= .5 and .5*cruise_speed <= horizontal_speed <= 1.5*cruise_speed)


def project(point, origin, optical, width, height, hfov):
    local = mv(transpose(optical), sub(point, origin))
    if local[0] <= .05:
        return None
    focal = width / (2 * math.tan(math.radians(hfov) / 2))
    return (width / 2 - focal * local[1] / local[0],
            height / 2 - focal * local[2] / local[0])


def truth_box(target, size, origin, optical, width, height, hfov):
    corners = [project(add(target, corner), origin, optical, width, height, hfov)
               for corner in itertools.product((-size / 2, size / 2), repeat=3)]
    if any(corner is None for corner in corners):
        return None
    xs, ys = zip(*corners)
    # 纹理几何略突出立方体；保留少量边界，避免包含大块天空。
    x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
    padding = max(2., (x2 - x1) * .02)
    x1, y1, x2, y2 = x1-padding, y1-padding, x2+padding, y2+padding
    if x1 < 0 or y1 < 0 or x2 >= width or y2 >= height:
        return None
    return [x1, y1, x2-x1, y2-y1]


def bbox_iou(a, b):
    if a is None or b is None:
        return 0.
    intersect = max(0., min(a[0]+a[2], b[0]+b[2])-max(a[0], b[0])) * max(
        0., min(a[1]+a[3], b[1]+b[3])-max(a[1], b[1]))
    union = a[2]*a[3] + b[2]*b[3] - intersect
    return intersect / union if union > 0 else 0.


def tracked(t):
    return t.get('tracking', {}).get('state') == 'TRACK' and t.get('target', {}).get('valid', False)


class Session:
    def __init__(self, args):
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.stringmsg_pb2 import StringMsg
        self.args = args
        self.node = Node()
        self.lock = threading.RLock()
        self.telemetry = deque(maxlen=6000)
        self.joint_history = deque(maxlen=12000)
        self.poses = deque(maxlen=12000)
        self.images = deque(maxlen=8)
        self.latest = None
        self.latest_image_wall = 0.
        self.latest_pose_wall = 0.
        self.latest_telemetry_wall = 0.
        self.sequence = 0
        self.next_ping = 0.
        self.image_count = 0
        self.image_gap_count = 0
        self.duplicate_images = 0
        self.previous_image_stamp = None
        self.command_results = {}
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('0.0.0.0', 0))
        self.socket.setblocking(False)
        self.address = (socket.gethostbyname(args.host), args.udp_port)
        args.output.mkdir(parents=True, exist_ok=True)
        self.log = (args.output / 'tracking.jsonl').open('w')
        self.topics = [args.prefix+'/telemetry', args.prefix+'/image',
                       f'/world/{args.world}/dynamic_pose/info']
        for message, topic, callback in zip((StringMsg, Image, Pose_V), self.topics,
                                            (self.on_telemetry, self.on_image, self.on_pose)):
            if not self.node.subscribe(message, topic, callback):
                raise RuntimeError(f'无法订阅 {topic}')

    def record(self, kind, **data):
        with self.lock:
            self.log.write(json.dumps(dict(kind=kind, wall=time.time(), **data),
                                      ensure_ascii=False, allow_nan=False)+'\n')
            self.log.flush()
        if kind == 'event':
            print(json.dumps(data, ensure_ascii=False), flush=True)

    def on_pose(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec*1e-9
        for pose in message.pose:
            if pose.name != self.args.entity:
                continue
            p, q = pose.position, pose.orientation
            row = dict(stamp=stamp, position=(p.x, p.y, p.z), quaternion=(q.w, q.x, q.y, q.z))
            with self.lock:
                if not self.poses or stamp > self.poses[-1]['stamp']:
                    self.poses.append(row)
                    self.latest_pose_wall = time.monotonic()
            return

    def on_image(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec*1e-9
        frame = dict(stamp=stamp, width=message.width, height=message.height,
                     step=message.step, format=message.pixel_format_type, data=bytes(message.data))
        with self.lock:
            if self.previous_image_stamp is not None and stamp <= self.previous_image_stamp:
                self.duplicate_images += 1
                return
            self.images.append(frame)
            self.latest_image_wall = time.monotonic()
            self.image_count += 1
            if self.previous_image_stamp is not None and stamp-self.previous_image_stamp > .085:
                self.image_gap_count += 1
            self.previous_image_stamp = stamp

    def on_telemetry(self, message):
        try:
            t = json.loads(message.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            if not isinstance(t, dict) or 'ts_mono' not in t or 'joints_deg' not in t:
                return
            if self.latest is not None and t['ts_mono'] <= self.latest['ts_mono']:
                return
            self.telemetry.append(t)
            self.joint_history.append(t)
            self.latest = t
            self.latest_telemetry_wall = time.monotonic()

    def send(self, cmd, **kwargs):
        self.sequence += 1
        payload = dict(cmd=cmd, seq=self.sequence, **kwargs)
        self.socket.sendto(json.dumps(payload).encode(), self.address)
        self.record('command', payload=payload)
        return self.sequence

    def poll(self, seconds=.04):
        if time.monotonic() >= self.next_ping:
            self.send('ping')
            self.next_ping = time.monotonic()+.5
        while True:
            try:
                payload, source = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            if source != self.address:
                continue
            try:
                received = json.loads(payload)
            except ValueError:
                continue
            if received.get('type') != 'telemetry':
                self.record('udp', payload=received)
            if received.get('type') == 'command_result':
                self.command_results[received.get('seq')] = received
        time.sleep(seconds)

    def matched(self, t):
        if t.get('camera', {}).get('frame_seq') != t.get('frame', {}).get('seq'):
            return None
        stamp = t.get('frame', {}).get('sim_stamp', 0.)
        with self.lock:
            if not self.poses or not self.images:
                return None
            pose = interpolate_pose(self.poses, stamp)
            joints = interpolate_joints(self.joint_history, stamp)
            image = min(self.images, key=lambda item: abs(item['stamp']-stamp))
        if pose is None or joints is None or abs(image['stamp']-stamp) > .002:
            return None
        return pose, image, joints

    def wait_ready(self):
        deadline = time.monotonic()+self.args.wait_timeout
        while time.monotonic() < deadline:
            self.poll()
            with self.lock:
                t = self.latest
                poses = list(self.poses)
            if t is None or not poses or self.matched(t) is None:
                continue
            if t.get('protocol_version') != 4 or t.get('angle_convention') != 'upright_joint':
                raise AssertionError('需要两轴 v4 upright_joint 遥测')
            if t.get('mount', {}).get('static') is not False:
                raise AssertionError('动态载机姿态补偿未开启')
            if not stable_cruise(poses, self.args.altitude, self.args.cruise_speed):
                continue
            if not self.args.initialize and not tracked(t):
                continue
            self.record('event', event='ready', pose=poses[-1], automatic_initialize=self.args.initialize)
            return
        raise AssertionError('等待真实图像、动态载机遥测或稳定飞行超时')

    def initialize(self):
        """明确启用后仅一次对准和框选；return 后没有机械/目标指令。"""
        self.send('stop')
        deadline = time.monotonic()+45
        while time.monotonic() < deadline:
            self.poll(.08)
            with self.lock:
                t, pose = self.latest, self.poses[-1]
                cruise = stable_cruise(self.poses, self.args.altitude, self.args.cruise_speed)
            if not cruise:
                self.send('manual', yaw=0, pitch=0)
                continue
            desired = reachable_angles(pose, self.args.target)
            if not desired['reachable'] or desired['singular']:
                continue
            joints = t['joints_deg']
            ey, ep = desired['yaw']-joints['yaw']['pos'], desired['pitch']-joints['pitch']['pos']
            if abs(ey) < .6 and abs(ep) < .6:
                self.send('stop')
                self.poll(.1)
                break
            self.send('manual', yaw=max(-45., min(45., ey*4)), pitch=max(-45., min(45., ep*4)))
        else:
            self.send('stop')
            raise AssertionError('一次初始化未能对准可达靶标')
        # 停止机械对准后等待下一组严格同步的数据，避免异步回调的瞬时帧号差
        # 被误判为初始化失败；匹配标准与后续跟踪评分保持一致。
        deadline = time.monotonic()+2
        matched = None
        while time.monotonic() < deadline:
            self.poll(.02)
            with self.lock:
                t = self.latest
            matched = self.matched(t)
            if matched is not None:
                break
        if matched is None:
            raise AssertionError('框选初始化时没有同步图像/位姿')
        pose, image, frame_joints = matched
        origin, optical = camera_pose(pose, frame_joints)
        box = truth_box(self.args.target, self.args.target_size, origin, optical,
                        image['width'], image['height'], self.args.hfov)
        if box is None or min(box[2:]) < 12:
            raise AssertionError('初始化靶标未完整出现在画面中或小于12px')
        self.save_snapshot('initial_selection.png', image, t, pose, frame_joints, box)
        norm = [box[0]/image['width'], box[1]/image['height'],
                box[2]/image['width'], box[3]/image['height']]
        requested_at = t['ts_mono']
        request_seq = self.send('track', box=norm)
        self.record('event', event='single_initial_selection', box_norm=norm,
                    note='初始化结束；后续目标真值仅用于评分，控制仅由 CSRT 和载机姿态补偿处理')
        deadline = time.monotonic()+12
        while time.monotonic() < deadline:
            self.poll()
            receipt = self.command_results.get(request_seq)
            if receipt is not None and not receipt.get('ok'):
                raise AssertionError(f'UDP track 执行失败：{receipt}')
            with self.lock:
                if receipt is not None and receipt.get('ok') and self.latest['ts_mono'] > requested_at and tracked(self.latest):
                    return
        raise AssertionError('UDP 框选缺少对应成功回执或未进入新的真实 CSRT TRACK')

    def sample(self, t):
        matched = self.matched(t)
        if matched is None:
            return None
        pose, image, frame_joints = matched
        origin, optical = camera_pose(pose, frame_joints)
        target_vector = sub(self.args.target, origin)
        los = mv(optical, (1., 0., 0.))
        # 相机遥测是 ts_mono 当前值，与图像时刻的框单独评分。
        with self.lock:
            current_pose = interpolate_pose(self.poses, t['ts_mono'])
        if current_pose is None:
            return None
        _, current_optical = camera_pose(current_pose, t)
        telemetry_los = mv(rotation(t['camera']['quat_world']), (1., 0., 0.))
        truth = truth_box(self.args.target, self.args.target_size, origin, optical,
                          image['width'], image['height'], self.args.hfov)
        box = t.get('target', {}).get('box_px')
        ray_error = None
        if box is not None:
            focal = image['width']/(2*math.tan(math.radians(self.args.hfov)/2))
            bx, by, bw, bh = box
            ray = mv(optical, (1., -(bx+bw/2-image['width']/2)/focal,
                               -(by+bh/2-image['height']/2)/focal))
            ray_error = angle_deg(ray, target_vector)
        reach = reachable_angles(current_pose, self.args.target)
        return dict(stamp=t['frame']['sim_stamp'], frame_seq=t['frame']['seq'],
                    position_enu=pose['position'], state=t['tracking']['state'],
                    quaternion_world=pose['quaternion'],
                    track=tracked(t), target_ray_error_deg=ray_error,
                    optical_axis_error_deg=angle_deg(los, target_vector),
                    camera_rotation_disagreement_deg=angle_deg(mv(current_optical, (1., 0., 0.)), telemetry_los),
                    center_error_deg=t.get('stability', {}).get('center_err_deg'),
                    target_bbox_iou=bbox_iou(box, truth), truth_visible=truth is not None,
                    reachable=reach['reachable'], truth_singular=reach['singular'],
                    singular=t.get('singular', False), limits=t.get('limits', {}),
                    constraint=t['tracking'].get('constraint', 'MISSING'),
                    tracking_valid_frames=t['tracking'].get('valid_frames', 0),
                    tracking_invalid_frames=t['tracking'].get('invalid_frames', 0),
                    pose_frame_delta_s=pose['stamp']-t['frame']['sim_stamp'],
                    joints_frame_delta_s=frame_joints['ts_mono']-t['frame']['sim_stamp'],
                    telemetry_image_age_sim_s=t['ts_mono']-t['frame']['sim_stamp'],
                    image_frame_delta_s=image['stamp']-t['frame']['sim_stamp'])

    def save_snapshot(self, name, image, t, pose, frame_joints, box=None):
        from PIL import Image, ImageDraw
        modes = {1: ('L', 'L', 1), 3: ('RGB', 'RGB', 3), 4: ('RGBA', 'RGBA', 4),
                 5: ('RGBA', 'BGRA', 4), 8: ('RGB', 'BGR', 3)}
        if image['format'] not in modes:
            raise ValueError(f'截图不支持像素格式 {image["format"]}')
        mode, rawmode, channels = modes[image['format']]
        picture = Image.frombytes(mode, (image['width'], image['height']), image['data'],
                                  'raw', rawmode, image['step'] or image['width']*channels).convert('RGB')
        draw = ImageDraw.Draw(picture)
        origin, optical = camera_pose(pose, frame_joints)
        expected = truth_box(self.args.target, self.args.target_size, origin, optical,
                             image['width'], image['height'], self.args.hfov)
        for value, color, label in ((expected, 'yellow', 'independent target projection'),
                                    (box or t.get('target', {}).get('box_px'), 'lime', 'CSRT / selection')):
            if value is not None:
                x, y, w, h = value
                draw.rectangle((x, y, x+w, y+h), outline=color, width=3)
                draw.text((x, max(0, y-15)), label, fill=color)
        draw.rectangle((0, 0, image['width'], 32), fill='black')
        draw.text((8, 8), f"sim={image['stamp']:.2f}s state={t['tracking']['state']} "
                  f"constraint={t['tracking'].get('constraint', 'MISSING')} "
                  f"ENU={tuple(round(v, 1) for v in pose['position'])}", fill='white')
        picture.save(self.args.output/name)

    def run(self):
        self.wait_ready()
        if self.args.initialize:
            self.initialize()
        with self.lock:
            start_sim = self.latest['frame']['sim_stamp']
            self.telemetry.clear()
            start_images, start_gaps = self.image_count, self.image_gap_count
            start_duplicates = self.duplicate_images
        start_wall = time.monotonic()
        samples = []
        last_seq = None
        last_snapshot = -math.inf
        seen_frames, accepted_frames, mismatched_frames = set(), set(), set()
        seen_states = set()
        while True:
            self.poll()
            with self.lock:
                pending = list(self.telemetry)
                self.telemetry.clear()
                latest = self.latest
                stale_image = time.monotonic()-self.latest_image_wall > 5
                stale_pose = time.monotonic()-self.latest_pose_wall > 5
                stale_telemetry = time.monotonic()-self.latest_telemetry_wall > 5
            if stale_image or stale_pose or stale_telemetry:
                raise AssertionError('真实图像、Gazebo载机位姿或插件遥测超过5墙钟秒未更新')
            for t in pending:
                seq = t.get('frame', {}).get('seq')
                if seq is None or seq == last_seq:
                    continue
                seen_frames.add(seq)
                if t.get('camera', {}).get('frame_seq') != seq:
                    mismatched_frames.add(seq)
                row = self.sample(t)
                if row is None:
                    continue
                last_seq = seq
                accepted_frames.add(seq)
                samples.append(row)
                self.record('sample', **row)
                state = (row['state'], row['constraint'])
                if row['stamp']-last_snapshot >= self.args.snapshot_interval or state not in seen_states:
                    pose, image, frame_joints = self.matched(t)
                    self.save_snapshot(f'frame_{len(samples):06d}.png', image, t, pose, frame_joints)
                    last_snapshot = row['stamp']
                    seen_states.add(state)
                if not -90.25 <= t['joints_deg']['yaw']['pos'] <= 90.25 or not -45.25 <= t['joints_deg']['pitch']['pos'] <= 90.25:
                    raise AssertionError('实际关节角超出机械限位')
            sim_elapsed = latest['frame']['sim_stamp']-start_sim
            wall_elapsed = time.monotonic()-start_wall
            elapsed = sim_elapsed if self.args.timebase == 'simulation' else wall_elapsed
            if elapsed >= self.args.duration:
                break
            if wall_elapsed > self.args.wall_timeout:
                raise AssertionError('达到墙钟保护超时，仿真未完成要求的验收时长')
        with self.lock:
            image_count, image_gaps = self.image_count-start_images, self.image_gap_count-start_gaps
            image_duplicates = self.duplicate_images-start_duplicates
        report = summarize(samples, self.args)
        skipped = len(seen_frames-accepted_frames)
        coverage = len(accepted_frames)/len(seen_frames) if seen_frames else 0.
        if coverage < .9:
            report['failures'].append('真实图像、关节与位姿的同步覆盖率不足90%')
        if sim_elapsed < self.args.min_sim_duration:
            report['failures'].append('真实仿真观测时间不足，墙钟时长不能替代飞行时长')
        report['result'] = 'failed' if report['failures'] else 'passed'
        report.update(sim_seconds=sim_elapsed, wall_seconds=wall_elapsed, real_image_count=image_count,
                      image_stamp_gaps=image_gaps, duplicate_image_stamps=image_duplicates,
                      unsynchronized_frames=skipped, synchronized_frame_ratio=coverage,
                      processed_frame_mismatch_count=len(mismatched_frames),
                      target_enu=list(self.args.target), target_size_m=self.args.target_size,
                      initializer='one_initial_truth_projection' if self.args.initialize else 'manual_ground_station',
                      note='真值只用于一次可选初始化和独立验收；初始化后未发布任何控制指令')
        (self.args.output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        self.record('event', event='result', **report)
        return report

    def close(self):
        try:
            self.send('bye')
        finally:
            self.socket.close()
            for topic in self.topics:
                self.node.unsubscribe(topic)
            with self.lock:
                self.log.close()


def summarize(samples, args):
    failures = []
    if len(samples) < 30:
        failures.append('同步视频样本少于30帧')
    eligible = [r for r in samples if r['reachable'] and not r['truth_singular']]
    valid = [r for r in eligible if r['track']]
    independent = [r for r in valid if r['target_ray_error_deg'] is not None
                   and r['target_ray_error_deg'] <= args.max_truth_error
                   and r['target_bbox_iou'] >= .2]
    ratio = len(valid)/len(eligible) if eligible else 0.
    authentic_ratio = len(independent)/len(eligible) if eligible else 0.
    ray_p95 = percentile([r['target_ray_error_deg'] for r in valid])
    optical_p95 = percentile([r['optical_axis_error_deg'] for r in valid])
    center_p95 = percentile([r['center_error_deg'] for r in valid])
    consistency_p95 = percentile([r['camera_rotation_disagreement_deg'] for r in samples])
    frame_age_p95 = percentile([r['telemetry_image_age_sim_s'] for r in samples])
    positions = [r['position_enu'] for r in samples]
    distance = sum(length(sub(b, a)) for a, b in zip(positions, positions[1:]))
    north_span = max((p[1] for p in positions), default=0)-min((p[1] for p in positions), default=0)
    # 用累计方向距离滤掉悬停抖动；要求至少50m北行和50m南行。
    north = south = 0.
    speeds = []
    for a, b in zip(samples, samples[1:]):
        dt = b['stamp']-a['stamp']
        dy = b['position_enu'][1]-a['position_enu'][1]
        if dt > 0:
            speeds.append(length(sub(b['position_enu'], a['position_enu']))/dt)
        if dt <= 0 or abs(dy/dt) < 2:
            continue
        if dy > 0:
            north += dy
        else:
            south -= dy
    constrained = [r for r in samples if not r['reachable'] or r['truth_singular']
                   or r['singular'] or r['limits'].get('yaw') or r['limits'].get('pitch')]
    unannounced = [r for r in constrained if r['constraint'] in ('MISSING', 'NONE')]
    states = Counter(r['state'] for r in samples)
    reference = samples[0]['quaternion_world'] if samples else (1., 0., 0., 0.)
    attitude_excursion = max((math.degrees(2*math.acos(min(1., abs(dot(reference, r['quaternion_world'])))))
                             for r in samples), default=0.)
    observed_seconds = samples[-1]['stamp']-samples[0]['stamp'] if samples else 0.
    largest_frame_gap = max((b['stamp']-a['stamp'] for a, b in zip(samples, samples[1:])), default=0.)
    if ratio < args.min_track_ratio:
        failures.append(f'可达期间 CSRT TRACK 比例 {ratio:.1%} 低于要求')
    if authentic_ratio < args.min_track_ratio:
        failures.append(f'独立靶标确认比例 {authentic_ratio:.1%} 低于要求，可能错框或丢失')
    if ray_p95 is None or ray_p95 > args.max_truth_error:
        failures.append('CSRT框中心与独立靶标方向P95误差超标')
    if optical_p95 is None or optical_p95 > args.max_center_error:
        failures.append('真实相机光轴与靶标方向P95误差超标')
    if consistency_p95 is None or consistency_p95 > 1:
        failures.append('独立载机/关节重建与相机遥测方向不一致')
    if frame_age_p95 is None or frame_age_p95 > .2:
        failures.append('图像相对控制遥测的仿真延迟P95超过0.2秒')
    if north_span < args.min_north_span or north < args.min_direction_distance or south < args.min_direction_distance:
        failures.append('Gazebo真值未证明飞机完成足够距离的南北往返')
    if attitude_excursion < args.min_attitude_excursion:
        failures.append('实际载机姿态变化不足，未覆盖动态安装姿态补偿')
    if observed_seconds < args.min_sim_duration or (args.timebase == 'simulation' and observed_seconds < .95*args.duration):
        failures.append('同步视频样本未覆盖要求的真实仿真观测时长')
    if largest_frame_gap > .5:
        failures.append('同步真实图像存在超过0.5仿真秒的观测空档')
    if unannounced:
        failures.append('不可达、退化或限位期间有样本未明确显示约束状态')
    if any(r['constraint'] == 'MISSING' for r in samples):
        failures.append('遥测缺少 tracking.constraint 状态')
    return dict(result='passed' if not failures else 'failed', failures=failures,
                samples=len(samples), reachable_frames=len(eligible), csrt_track_ratio=ratio,
                independently_confirmed_ratio=authentic_ratio,
                p95_target_ray_error_deg=ray_p95, p95_optical_axis_error_deg=optical_p95,
                p95_center_error_deg=center_p95, p95_camera_rotation_disagreement_deg=consistency_p95,
                p95_image_age_sim_s=frame_age_p95,
                actual_distance_m=distance, north_span_m=north_span,
                north_travel_m=north, south_travel_m=south,
                p95_speed_mps=percentile(speeds), attitude_excursion_deg=attitude_excursion,
                synchronized_sim_seconds=observed_seconds, max_observation_gap_sim_s=largest_frame_gap,
                height_range_m=[min((p[2] for p in positions), default=0),
                                max((p[2] for p in positions), default=0)],
                tracking_states=dict(states), constraint_states=dict(Counter(r['constraint'] for r in samples)),
                lost_frames=sum(states[state] for state in ('LOST', 'TEMP_LOST')),
                constrained_frames=len(constrained), unannounced_constraint_frames=len(unannounced),
                csrt_valid_counter_delta=max((r['tracking_valid_frames'] for r in samples), default=0)-min(
                    (r['tracking_valid_frames'] for r in samples), default=0),
                csrt_invalid_counter_delta=max((r['tracking_invalid_frames'] for r in samples), default=0)-min(
                    (r['tracking_invalid_frames'] for r in samples), default=0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--initialize', action='store_true', help='明确允许一次机械对准和视频框选')
    mode.add_argument('--manual-only', action='store_true', help='只观察人工地面站框选（默认）')
    parser.add_argument('--duration', type=float, default=120., help='验收时长，默认120仿真秒')
    parser.add_argument('--timebase', choices=('simulation', 'wall'), default='simulation')
    parser.add_argument('--wall-timeout', type=float, default=1800.)
    parser.add_argument('--wait-timeout', type=float, default=900., help='等待图像/稳定飞行的墙钟秒')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--udp-port', '--port', type=int, default=8770)
    parser.add_argument('--prefix', default='/rocket_two_axis_pod')
    parser.add_argument('--world', default=WORLD)
    parser.add_argument('--entity', default=ENTITY)
    parser.add_argument('--target', type=float, nargs=3, default=TARGET_POSITION)
    parser.add_argument('--target-size', type=float, default=TARGET_SIZE)
    parser.add_argument('--altitude', type=float, default=80., help='稳定飞行世界ENU高度近似值')
    parser.add_argument('--cruise-speed', type=float, default=15., help='初始化巡航速度判据（m/s）')
    parser.add_argument('--hfov', type=float, default=60.)
    parser.add_argument('--snapshot-interval', type=float, default=10., help='真实图像截图间隔（仿真秒）')
    parser.add_argument('--min-track-ratio', type=float, default=.95)
    parser.add_argument('--max-truth-error', type=float, default=3.)
    parser.add_argument('--max-center-error', type=float, default=3.)
    parser.add_argument('--min-north-span', type=float, default=180.)
    parser.add_argument('--min-direction-distance', type=float, default=50.)
    parser.add_argument('--min-sim-duration', type=float, default=40., help='墙钟观察也需至少此仿真秒数')
    parser.add_argument('--min-attitude-excursion', type=float, default=5., help='要求载机实际姿态变化角度（deg）')
    parser.add_argument('--output', type=Path, default=ROOT/'runs/geographic_rocket_two_axis_pod/tracking')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (*args.target, args.target_size, args.altitude, args.duration,
                                        args.wall_timeout, args.wait_timeout, args.hfov, args.snapshot_interval,
                                        args.min_track_ratio, args.max_truth_error, args.max_center_error,
                                        args.min_north_span, args.min_direction_distance, args.cruise_speed,
                                        args.min_sim_duration, args.min_attitude_excursion)):
        parser.error('数值参数必须有限')
    if min(args.duration, args.wall_timeout, args.wait_timeout, args.target_size, args.snapshot_interval,
           args.cruise_speed, args.min_sim_duration, args.max_truth_error, args.max_center_error,
           args.min_north_span, args.min_direction_distance, args.min_attitude_excursion) <= 0:
        parser.error('时长、靶标尺寸和截图间隔必须为正')
    if not 0 < args.hfov < 180 or not 0 < args.min_track_ratio <= 1 or not 1 <= args.udp_port <= 65535:
        parser.error('hfov需(0,180)，比例需(0,1]，UDP端口需[1,65535]')
    os.environ.setdefault('GZ_PARTITION', args.world)
    session = Session(args)
    try:
        report = session.run()
        return 0 if report['result'] == 'passed' else 1
    except Exception as error:
        if args.initialize:
            session.send('stop')
        report = dict(result='failed', error=str(error))
        (args.output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        session.record('event', event='failure', **report)
        return 1
    finally:
        session.close()


if __name__ == '__main__':
    raise SystemExit(main())
