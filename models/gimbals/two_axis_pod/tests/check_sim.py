#!/usr/bin/env python3
"""在线核对实际关节、光学姿态、渲染、限位与跟踪；需预先启动隔离仿真。"""
import argparse
import json
import math
from pathlib import Path
import threading
import time

from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node
from PIL import Image as PILImage


LIMITS = {"yaw": (-60., 60.), "pitch": (-80., 30.)}


def quaternion_product(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def quaternion_vector(q, v):
    norm = math.sqrt(sum(x*x for x in q))
    q = tuple(x/norm for x in q)
    return quaternion_product(quaternion_product(q, (0., *v)),
                              (q[0], -q[1], -q[2], -q[3]))[1:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, default=Path('runs/two_axis_pod'))
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    node = Node()
    lock = threading.Lock()
    tel, results, frames, poses = {}, {}, {}, {}
    samples, optical_checks, frame_checks = [], [], []

    def telemetry(message):
        with lock:
            tel.clear()
            tel.update(json.loads(message.data))

    def result(message):
        value = json.loads(message.data)
        with lock:
            results[value.get('request_id')] = value

    def image(kind):
        def callback(message):
            with lock:
                frames[kind] = (message.width, message.height, bytes(message.data), time.monotonic())
        return callback

    def pose(message):
        with lock:
            for value in message.pose:
                name = value.name.rsplit('::', 1)[-1]
                if name in ('camera_link', 'yaw_arm_link'):
                    q = value.orientation
                    poses[name] = ((q.w, q.x, q.y, q.z), time.monotonic())

    node.subscribe(StringMsg, '/two_axis_pod/telemetry', telemetry)
    node.subscribe(StringMsg, '/two_axis_pod/command_result', result)
    node.subscribe(Pose_V, '/world/two_axis_pod_demo/dynamic_pose/info', pose)
    callbacks = [image('camera'), image('overview')]
    node.subscribe(Image, '/world/two_axis_pod_demo/model/two_axis_pod/link/camera_link/sensor/camera/image', callbacks[0])
    node.subscribe(Image, '/two_axis_pod/overview', callbacks[1])
    pub = node.advertise('/two_axis_pod/command', StringMsg)
    counter = 0

    def send(cmd, **kwargs):
        nonlocal counter
        counter += 1
        key = str(counter)
        message = StringMsg()
        message.data = json.dumps(dict(cmd=cmd, request_id=key, **kwargs))
        pub.publish(message)
        return key

    def snapshot():
        with lock:
            return json.loads(json.dumps(tel))

    def check_sample(value):
        assert value['protocol_version'] == 8 and value['angle_convention'] == 'motion_head_joint_right_positive', value
        assert set(value['joints_deg']) == {'yaw', 'pitch'}, value
        for axis, (lower, upper) in LIMITS.items():
            assert lower-.2 <= value['joints_deg'][axis]['pos'] <= upper+.2, value
        assert not value['singular'], '确认的机械范围内不应发生视线退化'

    def wait(predicate, seconds=12):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            value = snapshot()
            if value and 'joints_deg' in value:
                check_sample(value)
                if predicate(value):
                    return value
            time.sleep(.04)
        raise AssertionError('状态等待超时: '+str(snapshot()))

    def accepted(cmd, expected=True, **kwargs):
        key = send(cmd, **kwargs)
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            with lock:
                value = results.get(key)
            if value is not None:
                assert value['ok'] == expected, value
                return value
            time.sleep(.02)
        raise AssertionError('缺少指令回执 '+cmd)

    def stopped():
        before = snapshot().get('ts_mono', -1)
        accepted('stop')
        return wait(lambda value: value['ts_mono'] > before and value['mode'] == 'IDLE' and
                    all(abs(value['joints_deg'][axis]['vel']) < .2 for axis in LIMITS))

    def home():
        before = snapshot().get('ts_mono', -1)
        accepted('home')
        return wait(lambda value: value['ts_mono'] > before and value['mode'] == 'IDLE' and all(
            abs(value['joints_deg'][axis]['pos']) < .15 and abs(value['joints_deg'][axis]['vel']) < .2
            for axis in LIMITS), 15)

    def actual_optics(label, value):
        # 活动 link 姿态由 SceneBroadcaster 独立提供；演示模型固定且世界旋转为单位阵。
        deadline = time.monotonic()+4
        observed = None
        while time.monotonic() < deadline:
            with lock:
                observed = poses.get('camera_link')
            if observed and time.monotonic()-observed[1] < 1:
                break
            time.sleep(.04)
        assert observed is not None, '未收到实际 camera_link 姿态'
        sensor_q = (math.sqrt(.5), 0., -math.sqrt(.5), 0.)
        actual_q = quaternion_product(observed[0], sensor_q)
        reported_q = value['camera']['quat_world']
        similarity = abs(sum(a*b for a, b in zip(actual_q, reported_q)))
        assert similarity > 1-1e-5, (label, actual_q, reported_q)
        actual_los = quaternion_vector(actual_q, (1., 0., 0.))
        yaw, pitch = (math.radians(value['joints_deg'][axis]['pos']) for axis in ('yaw', 'pitch'))
        expected = (-math.sin(pitch), -math.sin(yaw)*math.cos(pitch), math.cos(yaw)*math.cos(pitch))
        assert max(abs(a-b) for a, b in zip(actual_los, expected)) < .003, (label, actual_los, expected)
        assert max(abs(a-b) for a, b in zip(actual_los, value['los_mount'])) < .003, value
        optical_checks.append(dict(label=label, joints_deg=value['joints_deg'], actual_quat_world=actual_q,
                                   actual_los_mount=actual_los, reported_los_mount=value['los_mount']))
        samples.append(value)
        return actual_los

    def save_frames(label):
        before = time.monotonic()
        deadline = before+10
        while time.monotonic() < deadline:
            with lock:
                saved = dict(frames)
            if len(saved) == 2 and all(frame[3] > before for frame in saved.values()):
                break
            time.sleep(.05)
        assert len(saved) == 2 and all(frame[3] > before for frame in saved.values()), '两路相机未提供新鲜画面'
        for name, (width, height, data, _) in saved.items():
            step = max(1, len(data)//4096)
            levels = len(set(data[::step]))
            assert width > 0 and height > 0 and len(data) == width*height*3, f'{name} 图像数据不完整'
            # 端点视线可能只包含均匀天空；零位的纹理靶标和近景用于确认实际渲染。
            if label == 'zero' or name == 'overview':
                assert levels > 3, f'{name} 未完成有效渲染'
            frame_checks.append(dict(label=label, camera=name, width=width, height=height,
                                     sampled_color_levels=levels))
            PILImage.frombytes('RGB', (width, height), data).save(args.output/f'{name}_{label}.png')
        return list(saved)

    def drive(axis, rate, target):
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            send('manual', **{axis: rate})
            value = snapshot()
            check_sample(value)
            position = value['joints_deg'][axis]['pos']
            if abs(position-target) < .4 and value['limits'][axis]:
                stopped_value = stopped()
                return stopped_value['joints_deg'][axis]['pos'], stopped_value
            time.sleep(.1)
        raise AssertionError(f'{axis}未到限位 {target}: {snapshot()}')

    t = wait(lambda value: 'joints_deg' in value)
    accepted('reset_axis', expected=False, axis='roll')
    accepted('manual', expected=False, roll=0)
    t = home()
    actual_optics('zero', t)
    rendered = save_frames('zero')
    accepted('track', box=[.41, .34, .18, .32])
    t = wait(lambda value: value['tracking']['state'] == 'TRACK' and value['target']['valid'], 8)
    assert t['tracking']['valid_frames'] > 0 and t['target']['miss']['valid'], t
    samples.append(t)
    stopped()
    t = home()
    assert t['los_mount'][2] > .999 and not t['singular'], t
    for _ in range(6):
        send('manual', yaw=30)
        time.sleep(.1)
    t = stopped()
    assert t['joints_deg']['yaw']['pos'] > 2 and t['los_mount'][1] < -.03 and t['los_mount'][2] < .999, t
    actual_optics('yaw_motion', t)
    positions = {}
    for axis, rate, target in [('yaw', 80, 60), ('yaw', -80, -60),
                                ('pitch', -80, -80), ('pitch', 80, 30)]:
        home()
        position, t = drive(axis, rate, target)
        positions[f'{axis}_{target}'] = position
        actual_optics(f'{axis}_{target}', t)
        save_frames(f'{axis}_{target}')
        if axis == 'pitch' and target == -80:
            assert t['los_mount'][0] > .98, t
    home()
    send('manual', yaw=20, pitch=20)
    wait(lambda value: value['mode'] == 'MANUAL')
    t = wait(lambda value: value['mode'] == 'IDLE' and
             all(abs(value['cmd_dps'][axis]) < .001 for axis in LIMITS), 3)
    assert '超时' in (t['tracking'].get('message') or ''), t
    before = t['ts_mono']
    accepted('reset_axis', axis='yaw')
    after = wait(lambda value: value['ts_mono'] > before and value['mode'] == 'IDLE' and
                 abs(value['joints_deg']['yaw']['pos']) < .15)
    assert abs(after['joints_deg']['pitch']['pos']-t['joints_deg']['pitch']['pos']) < .2, (t, after)
    t = home()
    actual_optics('final_zero', t)
    stopped()
    report = dict(result='passed', protocol_version=8, angle_convention='motion_head_joint_right_positive',
                  actual_limits_deg=positions, rendered_frames=rendered, optical_checks=optical_checks,
                  frame_checks=frame_checks,
                  tracking='CSRT TRACK valid with target miss', watchdog='passed',
                  single_axis_reset='passed', zero_nonsingular_yaw_los_motion='passed', samples=samples)
    (args.output/'validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({key: value for key, value in report.items() if key not in ('samples', 'optical_checks', 'frame_checks')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
