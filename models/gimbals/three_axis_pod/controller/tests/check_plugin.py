#!/usr/bin/env python3
"""启动隔离 Gazebo 世界，注入图像并验证实际插件的超时与回执。"""
import json
import os
from pathlib import Path
import random
import signal
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

from gz.msgs10.image_pb2 import Image
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node

ROOT = Path(__file__).resolve().parents[5]


def main():
    run_dir = ROOT / 'runs/plugin_watchdog'
    run_dir.mkdir(parents=True, exist_ok=True)
    model = ET.parse(ROOT / 'models/gimbals/three_axis_pod/model.sdf').getroot().find('model')
    model.set('name', 'watchdog_pod')
    for link in model.findall('link'):
        for sensor in link.findall('sensor'):
            if sensor.get('name') == 'camera':
                topic = sensor.find('topic')
                if topic is None:
                    topic = ET.SubElement(sensor, 'topic')
                topic.text = '/watchdog/unused_camera'
                sensor.find('update_rate').text = '0.01'
            else:
                link.remove(sensor)
    joint = ET.SubElement(model, 'joint', name='test_anchor', type='fixed')
    ET.SubElement(joint, 'parent').text = 'world'
    ET.SubElement(joint, 'child').text = 'cgo3_mount_link'
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name='watchdog_test')
    ET.SubElement(world, 'gravity').text = '0 0 0'
    physics = ET.SubElement(world, 'physics', name='physics', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '0.004'
    ET.SubElement(physics, 'real_time_factor').text = '0.2'
    world.append(model)
    fixture = run_dir / 'world.sdf'
    ET.ElementTree(sdf).write(fixture, encoding='unicode')
    os.environ['GZ_PARTITION'] = 'watchdog_' + uuid.uuid4().hex
    os.environ['GZ_SIM_RESOURCE_PATH'] = str(ROOT / 'models/gimbals')
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(ROOT / 'models/gimbals/three_axis_pod/controller/build/lib')
    os.environ['GZ_SIM_SERVER_CONFIG_PATH'] = str(ROOT / 'models/gimbals/three_axis_pod/scripts/server.config')
    environment = dict(os.environ)
    environment.pop('DISPLAY', None)
    lock = threading.Lock()
    telemetry, results = [], {}

    def on_telemetry(msg):
        with lock:
            telemetry.append((time.monotonic(), json.loads(msg.data)))

    def on_result(msg):
        data = json.loads(msg.data)
        with lock:
            results[data.get('request_id')] = data

    def wait_for(predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with lock:
                value = predicate()
            if value:
                return value
            time.sleep(0.01)
        raise AssertionError('等待条件超时')

    with (run_dir / 'gazebo.log').open('w') as log:
        process = subprocess.Popen(['gz', 'sim', '-r', '-s', '-v2', str(fixture)],
                                   env=environment, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            node = Node()
            assert node.subscribe(StringMsg, '/gimbal/telemetry', on_telemetry)
            assert node.subscribe(StringMsg, '/gimbal/command_result', on_result)
            commands = node.advertise('/gimbal/command', StringMsg)
            images = node.advertise('/world/watchdog_test/model/watchdog_pod/link/'
                                    'camera_link/sensor/camera/image', Image)
            wait_for(lambda: telemetry, 15)

            def send(command, request_id=None, **parameters):
                message = StringMsg()
                message.data = json.dumps(dict(cmd=command, request_id=request_id,
                                               **parameters))
                assert commands.publish(message)

            send('home', 'first')
            send('home', 'second')
            wait_for(lambda: 'first' in results and 'second' in results)
            assert results['first']['ok'] and results['second']['ok']
            send('track', 'invalid')
            wait_for(lambda: 'invalid' in results)
            assert not results['invalid']['ok']
            print('PASS plugin request_id echo and correlated validation errors', flush=True)

            started = time.monotonic()
            # 初始镜头在天顶，只有公共 pitch 能确定地离开欧拉奇异区。
            send('manual', pitch=-30)
            wait_for(lambda: abs(telemetry[-1][1]['cmd_dps']['pitch']) > 0)
            stopped = wait_for(lambda: telemetry[-1] if telemetry[-1][1]['mode'] == 'IDLE'
                               and telemetry[-1][1]['cmd_dps']['yaw'] == 0 else None)
            age = stopped[0] - started
            assert 0.45 <= age <= 0.8, age
            print(f'PASS manual watchdog: {age:.3f}s at simulation factor 0.2', flush=True)

            pixels = bytearray(320 * 240 * 3)
            rng = random.Random(12345)
            for y in range(100, 132):
                for x in range(220, 252):
                    index = (y * 320 + x) * 3
                    pixels[index:index + 3] = bytes(rng.randrange(256) for _ in range(3))
            frame = Image(width=320, height=240, step=960, data=bytes(pixels))
            sequence = 0

            def publish_frame():
                nonlocal sequence
                sequence += 1
                frame.header.ClearField('data')
                entry = frame.header.data.add()
                entry.key = 'seq'
                entry.value.append(str(sequence))
                assert images.publish(frame)

            for _ in range(10):
                publish_frame()
                time.sleep(0.04)
            send('track', 'target', box=[220/320, 100/240, 32/320, 32/240])
            wait_for(lambda: 'target' in results)
            assert results['target']['ok'], results['target']
            for _ in range(10):
                publish_frame()
                time.sleep(0.04)
            wait_for(lambda: telemetry[-1][1]['tracking']['state'] == 'TRACK')

            target_pixels = frame.data
            for cycle in range(2):
                frame.data = bytes(320 * 240 * 3)
                occlusion_started = time.monotonic()
                while time.monotonic() - occlusion_started < 2.05:
                    publish_frame()
                    time.sleep(1 / 30)
                temporary = wait_for(lambda: telemetry[-1] if
                    telemetry[-1][1]['tracking']['state'] == 'TEMP_LOST' else None)
                assert not temporary[1]['target']['valid']
                assert all(v == 0 for v in temporary[1]['cmd_dps'].values())
                frame.data = target_pixels
                restored = time.monotonic()
                while time.monotonic() - restored < 0.2:
                    publish_frame()
                    time.sleep(1 / 30)
                wait_for(lambda: telemetry[-1][1]['tracking']['state'] == 'TRACK'
                         and telemetry[-1][1]['target']['valid'])
                print(f'PASS plugin occlusion {cycle + 1}: >2s continuous '
                      'blank frames, stopped output and automatic recovery', flush=True)

            last_frame = time.monotonic()
            stale = wait_for(lambda: telemetry[-1] if
                             telemetry[-1][1]['tracking']['state'] == 'TEMP_LOST' else None)
            assert all(v == 0 for v in stale[1]['cmd_dps'].values()), stale
            lost = wait_for(lambda: telemetry[-1] if
                            telemetry[-1][1]['tracking']['state'] == 'LOST' else None)
            assert 0.35 <= stale[0] - last_frame <= 0.8
            assert 1.8 <= lost[0] - last_frame <= 2.4
            assert not lost[1]['target']['valid']
            with lock:
                boxes = [t['target'] for _, t in telemetry if t['target']['box_px']]
            assert boxes and all(t['box_norm'] is not None for t in boxes)
            print(f'PASS video stop/lost: {stale[0]-last_frame:.3f}s/'
                  f'{lost[0]-last_frame:.3f}s; valid normalized boxes', flush=True)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


if __name__ == '__main__':
    main()
