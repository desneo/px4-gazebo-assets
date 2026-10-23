#!/usr/bin/env python3
"""独立物理世界：鼠标左右输入、实际光学姿态、机械遥测及艾晟 UDP 同号。

无渲染、无 PX4；创建独立 Gazebo 分区并仅停止本脚本启动的服务器。
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[2]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--plugin-dir', type=Path, default=ROOT/'controller/build/lib')
    ap.add_argument('--output', type=Path, default=REPO/'runs/two_axis_yaw_direction')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['GZ_PARTITION'] = 'yaw_right_' + uuid.uuid4().hex
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(args.plugin_dir.resolve())
    os.environ['GZ_SIM_RESOURCE_PATH'] = str(ROOT.parent)
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.transport13 import Node
    sys.path.insert(0, str(ROOT/'ground_station'))
    from ground_station import GroundStation
    from telemetry_forward import TelemetryForwarder
    from check_sim import quaternion_product, quaternion_vector

    def world_file(path, old_axis=False):
        sdf = ET.Element('sdf', version='1.9')
        world = ET.SubElement(sdf, 'world', name='yaw_direction')
        physics = ET.SubElement(world, 'physics', type='ode')
        ET.SubElement(physics, 'max_step_size').text = '.004'
        ET.SubElement(physics, 'real_time_factor').text = '1'
        ET.SubElement(world, 'gravity').text = '0 0 0'
        for filename, name in [('physics', 'Physics'), ('scene-broadcaster', 'SceneBroadcaster')]:
            ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
        model = copy.deepcopy(ET.parse(ROOT/'model.sdf').getroot().find('model'))
        world.append(model)
        mount = ET.SubElement(model, 'joint', name='world_mount', type='fixed')
        ET.SubElement(mount, 'parent').text = 'world'
        ET.SubElement(mount, 'child').text = 'mount_link'
        if old_axis:
            model.find("joint[@name='yaw_joint']/axis/xyz").text = '-1 0 0'
        ET.ElementTree(sdf).write(path, encoding='utf-8', xml_declaration=True)

    node = Node()
    lock = threading.Lock()
    telemetry, samples, poses = {}, [], {}
    forward = TelemetryForwarder()
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0)); receiver.settimeout(1.)
    forward.start('127.0.0.1', receiver.getsockname()[1])

    def on_telemetry(message):
        value = json.loads(message.data)
        with lock:
            telemetry.clear(); telemetry.update(value)
            samples.append(value)

    def on_pose(message):
        with lock:
            for pose in message.pose:
                if pose.name.rsplit('::', 1)[-1] == 'camera_link':
                    q = pose.orientation
                    poses['camera'] = ((q.w, q.x, q.y, q.z), time.monotonic())

    node.subscribe(StringMsg, '/two_axis_pod/telemetry', on_telemetry)
    node.subscribe(Pose_V, '/world/yaw_direction/dynamic_pose/info', on_pose)
    pub = node.advertise('/two_axis_pod/command', StringMsg)
    counter = 0

    def send(cmd, **values):
        nonlocal counter
        counter += 1
        message = StringMsg(); message.data = json.dumps(dict(cmd=cmd, request_id=str(counter), **values))
        pub.publish(message)

    def snapshot():
        with lock:
            return copy.deepcopy(telemetry)

    def wait(predicate, timeout=10.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            value = snapshot()
            if value and predicate(value):
                return value
            time.sleep(.025)
        raise AssertionError('等待遥测超时：' + repr(snapshot()))

    def home():
        send('home')
        time.sleep(.1)
        return wait(lambda t: t['mode'] == 'IDLE' and abs(t['joints_deg']['yaw']['pos']) < .15)

    def stop_server(server):
        if server.poll() is None:
            os.killpg(server.pid, signal.SIGINT)
            try:
                server.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(server.pid, signal.SIGTERM); server.wait(timeout=5)

    rows = []
    try:
        with tempfile.TemporaryDirectory(prefix='two-axis-yaw-') as directory:
            path = Path(directory)/'world.sdf'; world_file(path)
            with (args.output/'gazebo.log').open('w') as log:
                server = subprocess.Popen(['gz', 'sim', '-r', '-s', '-v', '2', str(path)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    wait(lambda t: 'attitude_deg' in t)
                    for side, sign in [('right', 1), ('left', -1)]:
                        home()
                        # 调用生产鼠标输入映射，随后将相同轴速送入真实插件。
                        held = {}
                        station = GroundStation.__new__(GroundStation)
                        station.display_size = (960, 540); station.joystick = 'aim'
                        station.manual = SimpleNamespace(hold=lambda axis, value, duration: held.update({axis: value}))
                        cx, cy, radius = station._joystick_geometry()[0]
                        station._update_joystick(cx + sign*radius, cy)
                        assert held['yaw'] == sign*20 and held['pitch'] == 0, held
                        started = time.monotonic()
                        while time.monotonic() - started < .8:
                            send('manual', **held); time.sleep(.08)
                        moving = wait(lambda t: t['joints_deg']['yaw']['vel']*sign > 1.)
                        assert moving['attitude_deg']['yaw']['pos']*sign > 1, moving
                        forward.offer(json.dumps(moving))
                        packet = json.loads(receiver.recv(8192))
                        assert packet['yaw_angle'] == moving['attitude_deg']['yaw']['pos']
                        assert 0 < packet['yaw_los_rate']*sign <= 20
                        send('stop'); time.sleep(.25)
                        stopped = wait(lambda t: t['mode'] == 'IDLE' and abs(t['joints_deg']['yaw']['vel']) < .1)
                        time.sleep(.1)
                        with lock:
                            actual, stamp = poses['camera']
                        assert time.monotonic() - stamp < 1.
                        optical = quaternion_product(actual, (math.sqrt(.5), 0., -math.sqrt(.5), 0.))
                        actual_los = quaternion_vector(optical, (1., 0., 0.))
                        # 零位图像右为安装 -Y；独立实体姿态必须与 yaw 符号一致。
                        assert -actual_los[1]*sign > .05, (side, actual_los)
                        assert max(abs(a-b) for a,b in zip(actual_los, stopped['los_mount'])) < .003
                        rows.append(dict(side=side, mouse_rate_dps=held['yaw'], moving=moving['attitude_deg']['yaw'],
                                         stopped_angle_deg=stopped['attitude_deg']['yaw']['pos'], actual_los_mount=actual_los, aisheng=packet))
                    home(); send('stop')
                finally:
                    stop_server(server)
            # 旧模型配新插件：启动时明确拒绝 -X，不能输出新协议遥测。
            time.sleep(.2)
            with lock:
                samples.clear(); telemetry.clear()
            world_file(path, old_axis=True)
            with (args.output/'old_axis.log').open('w') as log:
                server = subprocess.Popen(['gz', 'sim', '-r', '-s', '-v', '2', str(path)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    time.sleep(3.)
                    assert server.poll() is None, '旧轴向测试服务器意外退出'
                    assert not snapshot(), '旧轴向模型不应输出 v7 遥测'
                finally:
                    stop_server(server)
            assert '轴向不符合' in (args.output/'old_axis.log').read_text()
        report = dict(result='passed', protocol_version=7, angle_convention='motion_head_joint_right_positive',
                      checks=rows, old_axis_rejected=True, scope='独立物理世界，无渲染、无PX4、无整机飞行')
        files = ['parameters.json', 'model.sdf', 'tools/generate_model.py', 'controller/src/kinematics.cc',
                 'controller/src/gimbal_controller.cc', 'ground_station/ground_station.py', 'ground_station/telemetry_forward.py',
                 'tests/check_yaw_direction.py']
        report['source_sha256'] = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files}
        (args.output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False))
    finally:
        forward.close(); receiver.close()


if __name__ == '__main__':
    main()
