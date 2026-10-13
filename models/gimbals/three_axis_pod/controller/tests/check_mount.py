#!/usr/bin/env python3
"""在自定义地理 world 中验收固定安装的天空/水平控制和真实相机姿态。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
import zlib

ROOT = Path(__file__).resolve().parents[5]
SOURCE = ROOT / 'worlds/geo_32_217857_119_105878/world.sdf'
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'service'))
import numpy as np
import kinematics as kin
from gz.msgs10.image_pb2 import Image
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node


def save_png(path, frame):
    # Gazebo R8G8B8 每行可能含 stride，输出标准 RGB PNG。
    raw = b''.join(b'\0' + frame.data[y*frame.step:y*frame.step+frame.width*3]
                   for y in range(frame.height))
    def chunk(name, value):
        return struct.pack('!I', len(value)) + name + value + struct.pack('!I', zlib.crc32(name+value))
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', frame.width,
        frame.height, 8, 2, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mount-mode', choices=['skyward', 'downward'], required=True)
    parser.add_argument('--merged', action='store_true', help='通过 include merge=true 的装配验证安装外参')
    parser.add_argument('--public-manual', action='store_true', help='增加正负公共俯仰角速度与实际光学姿态校验')
    parser.add_argument('--plugin-dir', type=Path, default=Path(__file__).resolve().parents[1] / 'build/lib')
    args = parser.parse_args()
    mode = args.mount_mode
    run = ROOT / 'runs/mount_validation' / (mode + ('_merged' if args.merged else ''))
    run.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(SOURCE)
    world = tree.getroot().find('world')
    world.set('name', 'mount_validation')
    for tag in ('uri', 'albedo_map'):
        for node in world.iter(tag):
            if node.text and node.text.startswith('assets/'):
                node.text = str(SOURCE.parent / node.text)
    pod = ET.parse(ROOT / 'models/gimbals/three_axis_pod/model.sdf').getroot().find('model')
    pod.set('name', 'mount_pod')
    pose = pod.find('pose')
    if pose is None:
        pose = ET.SubElement(pod, 'pose')
    pose.text = f'0 0 50 {math.pi if mode == "downward" else 0} 0 0'
    pod.find('./plugin/mount_mode').text = mode
    if mode == 'downward':
        sensor_pose = pod.find('./link[@name="camera_link"]/sensor[@name="camera"]/pose')
        values = sensor_pose.text.split()
        sensor_pose.text = ' '.join(values[:3] + ['0', '-1.5707963267948966', '0'])
    if args.merged:
        # 外层模型姿态与安装 link 不同，用于检测误用模型根姿态。
        pose.text = '0 0 0 0 0 0'
        asset = ET.Element('sdf', version='1.9')
        asset.append(pod)
        asset_file = run / 'pod.sdf'
        ET.ElementTree(asset).write(asset_file, encoding='unicode')
        assembly = ET.Element('model', name='mount_pod')
        ET.SubElement(assembly, 'pose').text = '0 0 50 0 0 0'
        include = ET.SubElement(assembly, 'include', merge='true')
        ET.SubElement(include, 'uri').text = str(asset_file)
        ET.SubElement(include, 'pose').text = f'0 0 0 {math.pi if mode == "downward" else 0} 0 0'
        pod = assembly
    anchor = ET.SubElement(pod, 'joint', name='fixed_installation', type='fixed')
    ET.SubElement(anchor, 'parent').text = 'world'
    ET.SubElement(anchor, 'child').text = 'cgo3_mount_link'
    world.append(pod)
    fixture = run / 'world.sdf'
    tree.write(fixture, encoding='unicode')
    os.environ['GZ_PARTITION'] = 'mount_validation'
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(args.plugin_dir.resolve())
    os.environ['GZ_SIM_RESOURCE_PATH'] = str(ROOT / 'models/gimbals')
    latest = {}
    lock = threading.Lock()
    def record(key, value):
        with lock:
            latest[key] = value
    node = Node()
    assert node.subscribe(StringMsg, '/gimbal/telemetry', lambda msg: record('telemetry', json.loads(msg.data)))
    assert node.subscribe(StringMsg, '/gimbal/command_result', lambda msg: record('result', json.loads(msg.data)))
    prefix = '/world/mount_validation/model/mount_pod/link/camera_link/sensor/'
    assert node.subscribe(Image, prefix + 'camera/image', lambda msg: record('image', msg))
    assert node.subscribe(IMU, prefix + 'camera_imu/imu', lambda msg: record('imu', msg))
    command = node.advertise('/gimbal/command', StringMsg)
    def wait(predicate, timeout=45):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with lock:
                if predicate():
                    return
            time.sleep(.05)
        raise AssertionError('等待 Gazebo 验收条件超时: ' + str(list(latest)))
    records = []
    with (run / 'gazebo.log').open('w') as log:
        process = subprocess.Popen(['gz', 'sim', '-s', '-r', '-v2', str(fixture)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait(lambda: all(key in latest for key in ['telemetry', 'image', 'imu']))
            for operation in ['reset', 'home']:
                request_id = f'{mode}_{operation}_{time.monotonic_ns()}'
                msg = StringMsg(data=json.dumps({'cmd': operation, 'request_id': request_id}))
                command.publish(msg)
                wait(lambda: latest.get('result', {}).get('request_id') == request_id)
                assert latest['result']['ok'], latest['result']
                target_pitch = 0 if operation == 'reset' else (44 if mode == 'downward' else 90)
                wait(lambda: latest['telemetry']['mode'] == 'IDLE' and
                     abs(latest['telemetry']['attitude_deg']['pitch']['pos'] - target_pitch) < 0.8)
                time.sleep(.2)
                with lock:
                    telemetry, frame, imu = latest['telemetry'], latest['image'], latest['imu']
                q_imu = imu.orientation
                # 相机 IMU 相对 camera_link rpy=(0,0,3.14)，去除后乘真实光学传感器旋转。
                # Gazebo IMU 默认为初始传感器朝向参考，先还原世界参考系。
                initial_mount = kin.Rx(math.pi) if mode == 'downward' else np.eye(3)
                r_link = initial_mount @ kin.Rz(3.14) @ kin.quat_to_R(
                    [q_imu.w, q_imu.x, q_imu.y, q_imu.z]) @ kin.Rz(-3.14)
                r_sensor = r_link @ kin.sensor_rotation(mode)
                r_report = kin.quat_to_R(telemetry['camera']['quat_world'])
                angle = math.degrees(math.acos(float(np.clip((np.trace(r_sensor.T @ r_report)-1)/2, -1, 1))))
                los = r_report[:, 0]
                elevation = math.degrees(math.asin(float(np.clip(los[2], -1, 1))))
                (run / f'{operation}_diagnostic.json').write_text(json.dumps({'telemetry':telemetry,'imu':str(imu), 'r_sensor':r_sensor.tolist(),'r_report':r_report.tolist(),'angle':angle},indent=2))
                assert angle < 1, f'遥测与真实相机姿态偏差 {angle}'
                assert abs(sum(v*v for v in telemetry['camera']['quat_world'])-1) < 2e-5
                if operation == 'reset':
                    assert abs(elevation) < 1 and los[0] > .999, (elevation, los)
                    assert telemetry['angle_convention'] == 'px4_frd'
                    assert all(abs(telemetry['attitude_deg'][axis]['pos']) < 0.8
                               for axis in ['yaw', 'pitch', 'roll']), telemetry['attitude_deg']
                else:
                    assert abs(elevation - (44 if mode == 'downward' else 90)) < 1, elevation
                save_png(run / f'{operation}.png', frame)
                records.append({'operation': operation, 'world_elevation_deg': elevation,
                    'imu_telemetry_orientation_error_deg': angle, 'telemetry': telemetry})
                print(f'PASS {mode} {operation}: elevation={elevation:.3f}, imu error={angle:.4f}', flush=True)
                if operation == 'reset' and args.public_manual:
                    for sign in (1, -1):
                        before = latest['telemetry']['attitude_deg']['pitch']['pos']
                        until = time.monotonic() + 8
                        while time.monotonic() < until:
                            command.publish(StringMsg(data=json.dumps({'cmd': 'manual', 'pitch': sign * 10})))
                            time.sleep(.1)
                            if sign * (latest['telemetry']['attitude_deg']['pitch']['pos'] - before) >= 5:
                                break
                        stop_id = f'stop_{time.monotonic_ns()}'
                        command.publish(StringMsg(data=json.dumps({'cmd': 'stop', 'request_id': stop_id})))
                        wait(lambda: latest.get('result', {}).get('request_id') == stop_id)
                        wait(lambda: latest['telemetry']['mode'] == 'IDLE' and
                             all(abs(v) < .01 for v in latest['telemetry']['cmd_dps'].values()))
                        time.sleep(.2)
                        with lock:
                            actual_imu, actual_telemetry = latest['imu'], latest['telemetry']
                        iq = actual_imu.orientation
                        actual_link = initial_mount @ kin.Rz(3.14) @ kin.quat_to_R(
                            [iq.w, iq.x, iq.y, iq.z]) @ kin.Rz(-3.14)
                        actual_optical = actual_link @ kin.sensor_rotation(mode)
                        actual_pitch = math.degrees(math.asin(float(np.clip(actual_optical[2, 0], -1, 1))))
                        reported_pitch = actual_telemetry['attitude_deg']['pitch']['pos']
                        assert sign * (actual_pitch - before) > 4, (sign, before, actual_pitch)
                        assert abs(actual_pitch - reported_pitch) < 1, (actual_pitch, reported_pitch)
                        records.append({'operation': 'manual_pitch', 'direction': sign,
                            'before_pitch_deg': before, 'imu_pitch_deg': actual_pitch,
                            'reported_pitch_deg': reported_pitch})
                        print(f'PASS {mode} public pitch {sign:+}: IMU={actual_pitch:.3f}, reported={reported_pitch:.3f}', flush=True)
            (run / 'results.json').write_text(json.dumps(records, ensure_ascii=False, indent=2))
        finally:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


if __name__ == '__main__':
    main()
