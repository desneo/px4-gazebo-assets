#!/usr/bin/env python3
"""隔离物理验收：目标惯性 LOS、固定名义重心与生产艾晟 UDP。

使用原生吊舱和无重力简化载机，不启动 PX4 或图像渲染。合成帧只用于
启动生产 SIM_TRUTH/CSRT 后端；目标和载机由 Gazebo VelocityControl 运动。
SceneBroadcaster 世界位姿独立差分验证角速率，不使用插件给出的相对速度。
"""
import argparse
import bisect
import copy
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
import xml.etree.ElementTree as ET

POD = Path(__file__).resolve().parents[1]
REPO = POD.parents[2]
WORLD = 'inertial_target_los'
UAV = 'LOS_TEST_UAV'
TARGET = 'LOS_TEST_TARGET'
CG = (.3, -.2, .15)
INITIAL = (1.3, 2.8, 10.15)
SIZE = (640, 480)
PREFIX = '/inertial_los_check'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def text(parent, tag, value):
    item = parent.find(tag)
    if item is None:
        item = ET.SubElement(parent, tag)
    item.text = str(value)
    return item


def mass_center(model):
    """从零关节 SDF 惯性位姿求固定名义重心，仅用于构造和核对物理夹具。"""
    mass, moment = 0., [0., 0., 0.]
    for link in model.findall('link'):
        weight = float(link.findtext('inertial/mass'))
        pose = list(map(float, link.findtext('pose', '0 0 0 0 0 0').split()))
        x, y, z = map(float, link.findtext('inertial/pose', '0 0 0 0 0 0').split()[:3])
        roll, pitch, yaw = pose[3:]
        y, z = math.cos(roll)*y-math.sin(roll)*z, math.sin(roll)*y+math.cos(roll)*z
        x, z = math.cos(pitch)*x+math.sin(pitch)*z, -math.sin(pitch)*x+math.cos(pitch)*z
        x, y = math.cos(yaw)*x-math.sin(yaw)*y, math.sin(yaw)*x+math.cos(yaw)*y
        for axis, value in enumerate((x, y, z)):
            moment[axis] += weight * (pose[axis] + value)
        mass += weight
    return mass, [value / mass for value in moment]


def world_file(path, backend, cg=True):
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name=WORLD)
    physics = ET.SubElement(world, 'physics', type='ode')
    text(physics, 'max_step_size', '.004')
    text(physics, 'real_time_factor', '1')
    text(world, 'gravity', '0 0 0')
    for filename, name in [('physics', 'Physics'), ('scene-broadcaster', 'SceneBroadcaster'),
                           ('user-commands', 'UserCommands')]:
        plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system',
                               name=f'gz::sim::systems::{name}')
        if name == 'SceneBroadcaster':
            text(plugin, 'state_hertz', '250')
    model = copy.deepcopy(ET.parse(POD / 'model.sdf').find('model'))
    model.set('name', UAV)
    model.set('canonical_link', 'uav_body')
    pod_mass, pod_center = mass_center(model)
    body = ET.SubElement(model, 'link', name='uav_body')
    inertial = ET.SubElement(body, 'inertial')
    text(inertial, 'mass', '1000')
    # 夹具的真实零位合成重心就是声明的非零 CG，避免只命名一个任意观察偏移。
    body_center = [((1000 + pod_mass)*CG[axis] - pod_mass*pod_center[axis])/1000 for axis in range(3)]
    text(inertial, 'pose', ' '.join(format(value, '.17g') for value in body_center) + ' 0 0 0')
    inertia = ET.SubElement(inertial, 'inertia')
    for field in ('ixx', 'iyy', 'izz'):
        text(inertia, field, '1000')
    for field in ('ixy', 'ixz', 'iyz'):
        text(inertia, field, '0')
    joint = ET.SubElement(model, 'joint', name='uav_pod_mount', type='fixed')
    text(joint, 'parent', 'uav_body')
    text(joint, 'child', 'mount_link')
    plugin = model.find("plugin[@name='two_axis::GimbalController']")
    text(plugin, 'mount_dynamic', 'true')
    text(plugin, 'tracking_backend', backend)
    text(plugin, 'sim_truth_target_name', TARGET)
    text(plugin, 'los_target_name', TARGET)
    if cg:
        text(plugin, 'los_observer_cg_model', ' '.join(map(str, CG)))
    for field in ('image', 'command', 'telemetry', 'command_result'):
        text(plugin, field + '_topic', PREFIX + '/' + field)
    camera = model.find("link[@name='camera_link']/sensor[@name='camera']")
    text(camera, 'topic', PREFIX + '/image')
    world.append(model)
    target = ET.SubElement(world, 'model', name=TARGET)
    text(target, 'pose', ' '.join(map(str, INITIAL)) + ' 0 0 0')
    link = ET.SubElement(target, 'link', name='target_body')
    inertial = ET.SubElement(link, 'inertial')
    text(inertial, 'mass', '1')
    inertia = ET.SubElement(inertial, 'inertia')
    for field in ('ixx', 'iyy', 'izz'):
        text(inertia, field, '.1')
    for field in ('ixy', 'ixz', 'iyz'):
        text(inertia, field, '0')
    for element, name in ((model, 'uav'), (target, 'target')):
        velocity = ET.SubElement(element, 'plugin', filename='gz-sim-velocity-control-system',
                                  name='gz::sim::systems::VelocityControl')
        text(velocity, 'topic', PREFIX + '/' + name + '_velocity')
    assert max(abs(a-b) for a,b in zip(mass_center(model)[1], CG)) < 1e-12
    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(path, encoding='utf-8', xml_declaration=True)


class Fixture:
    def __init__(self, args, backend='sim_truth', cg=True):
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node
        from telemetry_forward import TelemetryForwarder
        self.Image, self.StringMsg, self.Twist = Image, StringMsg, Twist
        self.node = Node()
        self.lock = threading.RLock()
        self.latest, self.results, self.rows, self.poses = {}, {}, [], []
        self.stamp, self.current_poses, self.counter = 0., {}, 0
        self.errors, self.label = [], 'startup'
        self.running = True
        self.forward = TelemetryForwarder()
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver.bind(('127.0.0.1', 0))
        self.receiver.settimeout(2.)
        self.forward.start('127.0.0.1', self.receiver.getsockname()[1])
        self.node.subscribe(StringMsg, PREFIX + '/telemetry', self.on_telemetry)
        self.node.subscribe(StringMsg, PREFIX + '/command_result', self.on_result)
        self.node.subscribe(Pose_V, f'/world/{WORLD}/dynamic_pose/info', self.on_pose)
        self.command = self.node.advertise(PREFIX + '/command', StringMsg)
        self.image = self.node.advertise(PREFIX + '/image', Image)
        self.velocity_publishers = {name: self.node.advertise(PREFIX + '/' + name + '_velocity', Twist)
                                    for name in ('uav', 'target')}
        self.directory = args.output / (backend + ('_cg' if cg else '_no_cg'))
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = self.directory / 'world.sdf'
        world_file(self.path, backend, cg)
        self.log = (self.directory / 'gazebo.log').open('w')
        self.server = subprocess.Popen(['gz', 'sim', '-r', '-s', '-v', '2', str(self.path)],
                                       stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.frame_thread = threading.Thread(target=self.publish_frames, daemon=True)
        self.frame_thread.start()

    def on_telemetry(self, message):
        value = json.loads(message.data)
        with self.lock:
            self.latest = value
            self.rows.append(dict(kind='telemetry', phase=self.label, value=value))

    def on_result(self, message):
        value = json.loads(message.data)
        with self.lock:
            self.results[str(value.get('request_id'))] = value

    def on_pose(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
        with self.lock:
            for pose in message.pose:
                if pose.name not in (UAV, TARGET):
                    continue
                p, q = pose.position, pose.orientation
                self.current_poses[pose.name] = dict(position=[p.x, p.y, p.z], quaternion=[q.w, q.x, q.y, q.z])
            self.stamp = stamp
            if UAV in self.current_poses and TARGET in self.current_poses:
                row = dict(stamp=stamp, **copy.deepcopy(self.current_poses))
                self.poses.append(row)
                self.rows.append(dict(kind='pose', phase=self.label, value=row))

    def publish_frames(self):
        # 确定性纹理，仅用于验证绑定、数据链与真实后端执行，不替代光学渲染验收。
        width, height = SIZE
        data = bytes((x * 37 + y * 19 + channel * 71) % 256
                     for y in range(height) for x in range(width) for channel in range(3))
        seq = 0
        while self.running:
            with self.lock:
                stamp = self.stamp
            if stamp > 0:
                image = self.Image()
                image.width, image.height, image.step, image.pixel_format_type = width, height, width * 3, 3
                image.data = data
                image.header.stamp.sec = int(stamp)
                image.header.stamp.nsec = int(round((stamp - int(stamp)) * 1e9))
                entry = image.header.data.add()
                entry.key = 'seq'
                entry.value.append(str(seq))
                self.image.publish(image)
                seq += 1
            time.sleep(.04)

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.latest)

    def wait(self, predicate, timeout=12.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = self.snapshot()
            if value and predicate(value):
                return value
            if self.server.poll() is not None:
                raise AssertionError('Gazebo退出：' + str(self.directory / 'gazebo.log'))
            time.sleep(.02)
        raise AssertionError('状态等待超时：' + json.dumps(self.snapshot(), ensure_ascii=False))

    def send(self, cmd, **kwargs):
        self.counter += 1
        key = str(self.counter)
        message = self.StringMsg()
        message.data = json.dumps(dict(cmd=cmd, request_id=key, **kwargs))
        self.command.publish(message)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            with self.lock:
                result = self.results.get(key)
            if result is not None:
                assert result.get('ok'), result
                return result
            time.sleep(.01)
        raise AssertionError('指令无回执：' + cmd)

    def service(self, suffix, reqtype, request):
        result = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/{suffix}',
            '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
            '--req', request], capture_output=True, text=True, timeout=5, check=True)
        assert 'data: true' in result.stdout, result.stdout + result.stderr

    def set_pose(self, name, position):
        x, y, z = position
        self.service('set_pose', 'gz.msgs.Pose', f'name:"{name}" position:{{x:{x} y:{y} z:{z}}} orientation:{{w:1}}')

    def velocity(self, name, linear=(0., 0., 0.), angular=(0., 0., 0.)):
        message = self.Twist()
        message.linear.x, message.linear.y, message.linear.z = linear
        message.angular.x, message.angular.y, message.angular.z = angular
        self.velocity_publishers[name].publish(message)

    def ready(self):
        value = self.wait(lambda t: t.get('frame', {}).get('seq', -1) >= 3)
        assert value['protocol_version'] == 8, value
        return value

    def initialize(self, wrong_selection=False):
        self.velocity('target')
        self.velocity('uav')
        self.send('stop')
        self.set_pose(UAV, (0., 0., 0.))
        self.set_pose(TARGET, INITIAL)
        self.send('home')
        self.wait(lambda t: t['mode'] == 'IDLE' and all(abs(t['joints_deg'][axis]['pos']) < .12
                  and abs(t['joints_deg'][axis]['vel']) < .1 for axis in ('yaw', 'pitch')), 15)
        time.sleep(.25)
        # 零位相机朝+Z、图像右-Y/上-X，实体中心选框由固定外参独立求得。
        focal = SIZE[0] / (2 * math.tan(math.pi / 6))
        u = SIZE[0] / 2 - focal * INITIAL[1] / (INITIAL[2] - .0797)
        v = SIZE[1] / 2 + focal * (INITIAL[0] - .015) / (INITIAL[2] - .0797)
        box = [.73, .10, .12, .15] if wrong_selection else [(u-28)/SIZE[0], (v-28)/SIZE[1], 56/SIZE[0], 56/SIZE[1]]
        self.send('track', box=box)
        return self.wait(lambda t: t['mode'] == 'TRACK' and t['tracking']['state'] == 'TRACK')

    def packet(self, telemetry):
        self.forward.offer(json.dumps(telemetry, allow_nan=False))
        packet = json.loads(self.receiver.recv(65535))
        assert len(packet) == 16, packet
        assert packet['gyro_yaw_rate'] == max(-20., min(20., telemetry['attitude_deg']['yaw']['vel']))
        assert packet['gyro_pitch_rate'] == max(-20., min(20., telemetry['attitude_deg']['pitch']['vel']))
        los = telemetry['target_los']
        for axis in ('yaw', 'pitch'):
            expected = max(-20., min(20., round(los[axis + '_rate_dps'], 6))) if los['valid'] else 0.
            assert abs(packet[axis + '_los_rate'] - expected) < 1e-6, packet
        omega = los['omega_ned_dps'] if los['omega_valid'] else [0., 0., 0.]
        for axis, expected in zip(('n', 'e', 'd'), omega):
            assert abs(packet['los_omega_' + axis] - round(expected, 6)) < 1e-6, packet
        self.rows.append(dict(kind='udp', phase=self.label, telemetry=telemetry, packet=packet))
        return packet

    def close(self):
        self.running = False
        self.frame_thread.join(timeout=2)
        self.forward.close()
        self.receiver.close()
        for topic in self.node.subscribed_topics():
            self.node.unsubscribe(topic)
        if self.server.poll() is None:
            os.killpg(self.server.pid, signal.SIGINT)
            try:
                self.server.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(self.server.pid, signal.SIGTERM)
                self.server.wait(timeout=3)
        self.log.close()
        (self.directory / 'samples.json.gz').write_bytes(gzip.compress(
            json.dumps(self.rows, ensure_ascii=False, separators=(',', ':')).encode(), mtime=0))


def relative(row):
    from check_sim import quaternion_vector
    body = row[UAV]
    offset = quaternion_vector(body['quaternion'], CG)
    point = [body['position'][axis] + offset[axis] for axis in range(3)]
    r = [row[TARGET]['position'][axis] - point[axis] for axis in range(3)]
    return (r[1], r[0], -r[2])


def angles(r):
    return math.atan2(r[1], r[0]), math.atan2(-r[2], math.hypot(r[0], r[1]))


def pose_at(rows, stamp):
    index = bisect.bisect_left([row['stamp'] for row in rows], stamp)
    assert 0 < index < len(rows), ('独立位姿未覆盖采样时刻', stamp)
    before, after = rows[index-1], rows[index]
    if abs(after['stamp']-stamp) < 1e-8:
        return after
    dt = after['stamp'] - before['stamp']
    assert 0 < dt < .08, ('独立位姿间隔过大', dt)
    alpha = (stamp - before['stamp']) / dt
    result = dict(stamp=stamp)
    for name in (UAV, TARGET):
        p, q = before[name], after[name]
        sign = 1 if sum(a*b for a,b in zip(p['quaternion'], q['quaternion'])) >= 0 else -1
        result[name] = dict(position=[a + alpha*(b-a) for a,b in zip(p['position'], q['position'])],
                            quaternion=[a + alpha*(sign*b-a) for a,b in zip(p['quaternion'], q['quaternion'])])
    return result


def score(fixture, label, duration=.85, target_velocity=(0., 0., 0.),
          uav_velocity=(0., 0., 0.), uav_rotation=(0., 0., 0.), signs=None):
    fixture.initialize()
    fixture.label = label
    with fixture.lock:
        start = fixture.stamp
    fixture.velocity('target', target_velocity)
    fixture.velocity('uav', uav_velocity, uav_rotation)
    time.sleep(duration)
    fixture.velocity('target')
    fixture.velocity('uav')
    time.sleep(.1)
    with fixture.lock:
        poses = copy.deepcopy(fixture.poses)
        telemetry = [row['value'] for row in fixture.rows if row['kind'] == 'telemetry'
                     and row['phase'] == label and row['value']['target_los']['valid']]
    scored, max_position_error, max_rate_error, max_vector_error = [], 0., 0., 0.
    for sample in telemetry:
        los = sample['target_los']
        stamp = los['sim_stamp']
        if not start + .18 < stamp < start + duration - .08:
            continue
        current = pose_at(poses, stamp)
        before = pose_at(poses, stamp - .016)
        after = pose_at(poses, stamp + .016)
        r = relative(current)
        max_position_error = max(max_position_error, max(abs(a-b) for a,b in zip(r, los['relative_position_ned_m'])))
        a, b = angles(relative(before)), angles(relative(after))
        dt = after['stamp'] - before['stamp']
        truth = [math.degrees(math.remainder(b[0]-a[0], 2*math.pi))/dt,
                 math.degrees(b[1]-a[1])/dt]
        reported = [los['yaw_rate_dps'], los['pitch_rate_dps']]
        # 由独立世界位姿重建方位/仰角与其差分，不复用插件速度或被限幅的外发速率。
        psi, theta = angles(r)
        yaw, pitch = truth
        sp, cp, st, ct = math.sin(psi), math.cos(psi), math.sin(theta), math.cos(theta)
        omega_truth = [-pitch*sp + yaw*st*ct*cp,
                       pitch*cp + yaw*st*ct*sp, yaw*ct*ct]
        assert los['omega_valid'] is True
        omega = los['omega_ned_dps']
        max_vector_error = max(max_vector_error, max(abs(a-b) for a,b in zip(omega_truth, omega)))
        error = max(abs(a-b) for a,b in zip(truth, reported))
        max_rate_error = max(max_rate_error, error)
        packet = fixture.packet(sample)
        scored.append(dict(stamp=stamp, reported_dps=reported, truth_dps=truth,
                           omega_ned_dps=omega, omega_truth_dps=omega_truth, packet=packet,
                           mechanical_dps=[sample['attitude_deg'][axis]['vel'] for axis in ('yaw', 'pitch')]))
    assert len(scored) >= 8, (label, len(scored), fixture.snapshot().get('target_los'))
    assert max_position_error < .025, (label, max_position_error)
    assert max_rate_error < .35, (label, max_rate_error)
    assert max_vector_error < .35, (label, max_vector_error)
    means = [sum(row['reported_dps'][axis] for row in scored)/len(scored) for axis in range(2)]
    if signs:
        for axis, sign in signs.items():
            value = means[{'yaw': 0, 'pitch': 1}[axis]]
            assert value * sign > .1, (label, means)
    if label in ('static', 'radial'):
        assert max(abs(value) for row in scored for value in row['reported_dps']) < .025, (label, means)
    print(f'通过：{label}，{len(scored)}样本，独立角速度最大误差{max_rate_error:.6f}°/s', flush=True)
    return dict(label=label, samples=len(scored), mean_reported_dps=means,
                max_position_error_m=max_position_error, max_rate_error_dps=max_rate_error,
                max_vector_error_dps=max_vector_error,
                peak_raw_rate_dps=max(abs(value) for row in scored for value in row['reported_dps']),
                representative=scored[len(scored)//2])


def run(args):
    results, evidence = [], []
    fixture = Fixture(args)
    try:
        initial = fixture.ready()
        fixture.packet(initial)
        assert initial['target_los']['valid'] is False and initial['target_los']['reason'] == 'target_unbound'
        results.append(score(fixture, 'static'))
        assert max(abs(a-b) for a,b in zip(results[-1]['representative']['reported_dps'],
                   results[-1]['representative']['mechanical_dps'])) > 1, '目标惯性LOS应与机械转速独立'
        results.append(score(fixture, 'yaw_right', target_velocity=(1., 0., 0.), signs={'yaw': 1}))
        results.append(score(fixture, 'yaw_left', target_velocity=(-1., 0., 0.), signs={'yaw': -1}))
        results.append(score(fixture, 'pitch_up', target_velocity=(0., 0., 1.), signs={'pitch': 1}))
        results.append(score(fixture, 'pitch_down', target_velocity=(0., 0., -1.), signs={'pitch': -1}))
        results.append(score(fixture, 'radial', target_velocity=(.2, .6, 2.)))
        results.append(score(fixture, 'observer_east', uav_velocity=(1., 0., 0.), signs={'yaw': -1}))
        results.append(score(fixture, 'observer_rotation_cg', uav_rotation=(0., 0., .4)))
        assert results[-1]['peak_raw_rate_dps'] > .2
        results.append(score(fixture, 'los_above_mechanical_limit', target_velocity=(8., 0., 0.)))
        assert results[-1]['peak_raw_rate_dps'] > 20
        assert any(abs(results[-1]['representative']['packet'][axis + '_los_rate']) == 20
                   for axis in ('yaw', 'pitch'))
        assert any(abs(results[-1]['representative']['packet']['los_omega_' + axis]) > 20
                   for axis in ('n', 'e', 'd')), '三维向量不得按机械限速裁剪'
        fixture.initialize()
        fixture.label = 'vertical_vector'
        fixture.set_pose(TARGET, (CG[0], CG[1], CG[2] + 10.))
        vertical = fixture.wait(lambda t: t['target_los']['reason'] == 'vertical_singularity'
                                and t['target_los']['omega_valid'])
        packet = fixture.packet(vertical)
        assert not vertical['target_los']['valid']
        assert packet['yaw_los_rate'] == packet['pitch_los_rate'] == 0.
        assert max(abs(packet['los_omega_' + axis]) for axis in ('n', 'e', 'd')) < .025
        results.append(dict(label='vertical_vector', passed=True,
                            omega_valid=True, angle_rates_valid=False, packet=packet))
        fixture.label = 'pause'
        fixture.initialize()
        fixture.wait(lambda t: t['target_los']['valid'])
        fixture.service('control', 'gz.msgs.WorldControl', 'pause:true')
        time.sleep(.2)
        paused = fixture.snapshot()
        time.sleep(.3)
        assert fixture.snapshot()['ts_mono'] == paused['ts_mono'], '暂停期间不应产生新的物理状态遥测'
        fixture.service('control', 'gz.msgs.WorldControl', 'pause:false')
        resumed = fixture.wait(lambda t: t['ts_mono'] > paused['ts_mono'] and t['target_los']['valid'])
        assert max(abs(resumed['target_los'][axis + '_rate_dps']) for axis in ('yaw', 'pitch')) < .025
        fixture.packet(resumed)
        results.append(dict(label='pause_resume', paused_no_new_telemetry=True, resume_no_rate_spike=True, passed=True))
        fixture.label = 'delete_target'
        fixture.service('remove', 'gz.msgs.Entity', f'name:"{TARGET}" type:MODEL')
        deleted = fixture.wait(lambda t: not t['target_los']['valid'] and t['target_los']['reason'] == 'target_missing')
        fixture.packet(deleted)
        results.append(dict(label='delete_target', reason=deleted['target_los']['reason'], passed=True))
        fixture.label = 'unbound_gyro_continues'
        fixture.send('manual', yaw=12.)
        moving = fixture.wait(lambda t: t['attitude_deg']['yaw']['vel'] > 8 and not t['target_los']['valid'])
        packet = fixture.packet(moving)
        assert packet['gyro_yaw_rate'] > 8 and packet['yaw_los_rate'] == packet['pitch_los_rate'] == 0
        results.append(dict(label='unbound_gyro_continues', packet=packet, passed=True))
    finally:
        fixture.close()
        evidence.extend(fixture.rows)
    fixture = Fixture(args, backend='csrt')
    try:
        fixture.ready()
        fixture.label = 'csrt_correct_binding'
        fixture.initialize()
        correct = fixture.wait(lambda t: t['target_los']['valid'])
        fixture.packet(correct)
        assert correct['target_los']['target_name'] == TARGET
        results.append(dict(label='csrt_correct_binding', passed=True))
        fixture.label = 'csrt_wrong_selection'
        fixture.initialize(wrong_selection=True)
        wrong = fixture.wait(lambda t: t['mode'] == 'TRACK' and t['tracking']['state'] == 'TRACK'
                             and t['target_los']['reason'] == 'target_selection_mismatch')
        fixture.packet(wrong)
        results.append(dict(label='csrt_wrong_selection', reason=wrong['target_los']['reason'],
                            control_mode=wrong['mode'], tracking_state=wrong['tracking']['state'], passed=True))
    finally:
        fixture.close()
        evidence.extend(fixture.rows)
    fixture = Fixture(args, cg=False)
    try:
        fixture.ready()
        fixture.label = 'no_aircraft_cg'
        fixture.initialize()
        invalid = fixture.wait(lambda t: t['target_los']['reason'] == 'observer_cg_unconfigured')
        fixture.packet(invalid)
        results.append(dict(label='no_aircraft_cg', reason=invalid['target_los']['reason'], passed=True))
    finally:
        fixture.close()
        evidence.extend(fixture.rows)
    return results, evidence


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--plugin-dir', type=Path, default=POD / 'controller/build/lib')
    ap.add_argument('--output', type=Path, default=REPO / 'runs/inertial_target_los')
    ap.add_argument('--archive', action='store_true', help='另保存新的 results/inertial_target_los.json 及压缩原始证据')
    args = ap.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    library = args.plugin_dir.resolve() / 'libgz-two-axis-pod-controller.so'
    if not library.is_file():
        ap.error('找不到控制库：' + str(library))
    os.environ['GZ_PARTITION'] = 'target_los_' + uuid.uuid4().hex
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(args.plugin_dir.resolve())
    os.environ['GZ_SIM_RESOURCE_PATH'] = str(POD.parent)
    sys.path.insert(0, str(POD / 'ground_station'))
    paths = [Path(__file__), POD / 'model.sdf', POD / 'ground_station/telemetry_forward.py',
             POD / 'ground_station/packet_rate.py']
    paths += [POD / 'controller' / path for path in (
        'CMakeLists.txt', 'src/gimbal_controller.cc', 'src/inertial_los.cc', 'src/control.cc',
        'src/truth_tracking.cc', 'src/kinematics.cc', 'service/telemetry.json',
        'include/two_axis/inertial_los.hh', 'include/two_axis/control.hh',
        'include/two_axis/truth_tracking.hh', 'include/two_axis/kinematics.hh',
        'include/two_axis/telemetry.hh', 'include/two_axis/control_schedule.hh')]
    fingerprints = {str(path.relative_to(REPO)): sha(path) for path in paths}
    cases, evidence = run(args)
    assert fingerprints == {str(path.relative_to(REPO)): sha(path) for path in paths}, '验收期间源文件变化，请重新运行'
    report = dict(passed=True, protocol_version=8, frame='NED', sign_convention='yaw_right_positive_pitch_up_positive',
                  observer_reference='aircraft_nominal_cg', observer_cg_model_m=CG,
                  observer_cg_definition='SDF零关节各link惯性位姿的质量加权合成；固连模型坐标',
                  source_sha256=fingerprints, controller_library_sha256=sha(library),
                  cases=cases, raw_rows=len(evidence),
                  scope='隔离简化载机与原生吊舱物理；SIM_TRUTH/CSRT绑定；生产艾晟UDP回环',
                  limitations=['无PX4整机飞行、无光学渲染与视频链验收', '目标/载机运动由Gazebo VelocityControl驱动',
                               '未覆盖真实网络丢包或不同主机时钟', '时间回退由纯算法单测覆盖，本物理验收覆盖暂停恢复'])
    encoded = json.dumps(dict(report=report, evidence=evidence), ensure_ascii=False, separators=(',', ':')).encode()
    compressed = gzip.compress(encoded, mtime=0)
    report['evidence_sha256'] = hashlib.sha256(compressed).hexdigest()
    (args.output / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    (args.output / 'samples.json.gz').write_bytes(compressed)
    if args.archive:
        (POD / 'tests/results/inertial_target_los.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        (POD / 'tests/results/inertial_target_los_samples.json.gz').write_bytes(compressed)
    print(f'目标惯性LOS验收通过：{len(cases)}组，原始{len(evidence)}行；{args.output / "summary.json"}')


if __name__ == '__main__':
    main()
