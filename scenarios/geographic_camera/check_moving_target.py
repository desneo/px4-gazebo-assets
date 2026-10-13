#!/usr/bin/env python3
"""验收 Gazebo 动态小车的往返和转弯跟踪，保存遥测与实际里程计。"""
import argparse
import json
import math
import os
from pathlib import Path
import socket
import threading
import time

from control_scene import set_pose
from scene_geometry import initialize_camera, target_x


def truth_error_deg(telemetry, odometry, mount_mode):
    """独立车顶真值与跟踪框视线角差；只用于验收，不回传给控制器。"""
    import numpy as np
    frame_time = telemetry.get('frame', {}).get('sim_stamp', telemetry['ts_mono'])
    point = min(odometry, key=lambda p: abs(p['sim_time'] - frame_time))
    if abs(point['sim_time'] - frame_time) > 0.2:
        raise AssertionError('车顶真值里程计与图像时刻相差超过 0.2 仿真秒')
    joints = telemetry['joints_deg']
    yaw, roll, pitch = [math.radians(joints[k].get('pos_raw', joints[k]['pos']))
                        for k in ('yaw', 'roll', 'pitch')]
    def rx(a):
        c, v = math.cos(a), math.sin(a)
        return np.array([[1, 0, 0], [0, c, -v], [0, v, c]])
    def ry(a):
        c, v = math.cos(a), math.sin(a)
        return np.array([[c, 0, v], [0, 1, 0], [-v, 0, c]])
    def rz(a):
        c, v = math.cos(a), math.sin(a)
        return np.array([[c, -v, 0], [v, c, 0], [0, 0, 1]])
    installation = {'skyward': np.eye(3), 'downward': rx(math.pi),
                    'sideward': ry(math.pi / 2)}[mount_mode]
    sensor = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    if mount_mode == 'downward':
        sensor = ry(-math.pi / 2)
    rotation = installation @ rz(-yaw) @ rx(-roll) @ ry(pitch) @ sensor
    x, y, width, height = telemetry['target']['box_px']
    focal = 960 / math.tan(math.pi / 6)
    ray = rotation @ np.array([1, -(x + width / 2 - 960) / focal,
                              -(y + height / 2 - 540) / focal])
    truth = np.array([point['x'], point['y'], point['z'] + 0.38 - 50])
    cosine = np.dot(ray, truth) / (np.linalg.norm(ray) * np.linalg.norm(truth))
    return math.degrees(math.acos(max(-1, min(1, float(cosine)))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--speed', type=float, default=0.5, help='车体前进速度，m/s')
    parser.add_argument('--leg-seconds', type=float, default=10, help='每段仿真秒数')
    parser.add_argument('--box', type=float, nargs=4, default=[0.465, 0.376, 0.072, 0.241])
    parser.add_argument('--output', type=Path, default=Path('runs/geographic_camera/moving_target.jsonl'))
    parser.add_argument('--mount-mode', choices=['skyward', 'downward', 'sideward'], default='skyward')
    args = parser.parse_args()
    if not 0 < args.speed <= 1 or not 5 <= args.leg_seconds <= 20:
        parser.error('速度范围 (0, 1] m/s，每段范围 [5, 20] 仿真秒')
    # partition 在导入 Transport 前设定，避免连接到其他仿真。
    os.environ.setdefault('GZ_PARTITION', 'geographic_camera')
    from gz.transport13 import Node
    from gz.msgs10.twist_pb2 import Twist
    from gz.msgs10.odometry_pb2 import Odometry
    node = Node()
    publisher = node.advertise('/model/tracking_target/cmd_vel', Twist)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log = args.output.open('w', encoding='utf-8')
    lock = threading.Lock()
    odometry = []

    def record(kind, **data):
        with lock:
            log.write(json.dumps(dict(wall_time=time.time(), monotonic=time.monotonic(), kind=kind, **data), ensure_ascii=False) + '\n')
            log.flush()
        if kind == 'event':
            print(data, flush=True)

    def on_odometry(message):
        position = message.pose.position
        point = dict(sim_time=message.header.stamp.sec + message.header.stamp.nsec * 1e-9,
                     x=position.x, y=position.y, z=position.z)
        odometry.append(point)
        record('odometry', **point)

    if not node.subscribe(Odometry, '/model/tracking_target/odometry', on_odometry):
        raise RuntimeError('无法订阅小车里程计')
    address = (socket.gethostbyname(args.host), args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 0))
    sock.settimeout(0.05)
    sequence = 0
    next_ping = 0
    last_telemetry = None

    def send(command, **parameters):
        nonlocal sequence
        sequence += 1
        payload = dict(cmd=command, seq=sequence, **parameters)
        sock.sendto(json.dumps(payload).encode(), address)
        record('command', payload=payload)

    def drive(speed=0, turn=0):
        message = Twist()
        message.linear.x = speed
        message.angular.z = turn
        publisher.publish(message)

    def collect(seconds, predicate=None, *, simulation=False, speed=0, turn=0):
        nonlocal next_ping, last_telemetry
        start = time.monotonic()
        sim_start = last_telemetry['ts_mono'] if last_telemetry else None
        samples = []
        while time.monotonic() - start < (max(30, seconds * 8) if simulation else seconds):
            now = time.monotonic()
            drive(speed, turn)
            if now >= next_ping:
                send('ping')
                next_ping = now + 0.5
            try:
                payload, source = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if source != address:
                continue
            data = json.loads(payload)
            record('received', payload=data)
            if data.get('type') == 'telemetry':
                last_telemetry = data
                samples.append(data)
                if predicate and predicate(data):
                    return samples
                if simulation:
                    if sim_start is None:
                        sim_start = data['ts_mono']
                    if data['ts_mono'] - sim_start >= seconds:
                        return samples
        if simulation:
            raise AssertionError('仿真时间未达到指定时长（遥测中断或实时率过低）')
        return samples

    def tracked(data):
        return data.get('tracking', {}).get('state') == 'TRACK' and bool(data.get('target', {}).get('valid'))

    try:
        record('event', event='start', speed_mps=args.speed, leg_sim_seconds=args.leg_seconds,
               minimum_track_ratio=0.95, maximum_p95_center_error_deg=3.0,
               maximum_p95_truth_error_deg=3.0)
        assert collect(30, predicate=lambda t: True), '没有收到遥测'
        drive()
        set_pose('occlusion_panel', 40, 0, 40)
        set_pose('tracking_target', target_x(args.mount_mode), 0, 30.6)
        initialize_camera(send, collect, args.mount_mode)
        collect(2, simulation=True)
        send('track', box=args.box)
        initial = collect(5, predicate=tracked)
        assert initial and tracked(initial[-1]), '目标框未初始化，请确认安装与 --box'
        collect(2, simulation=True)
        assert odometry, '没有小车里程计（请使用当前 prepare_world.py 生成的世界）'
        motion_start = len(odometry)
        samples = []
        for name, speed, turn in [('forward', args.speed, 0), ('reverse', -args.speed, 0),
                                   ('curve', args.speed, 0.12), ('curve_reverse', -args.speed, -0.12)]:
            record('event', event='segment', name=name, speed=speed, yaw_rate=turn)
            samples += collect(args.leg_seconds, simulation=True, speed=speed, turn=turn)
        drive()
        positions = odometry[motion_start:]
        assert positions, '运动期间没有里程计'
        # 使用发布的世界位姿检查实际运动及高度，独立对比车顶真值。
        distance = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(positions, positions[1:]))
        heights = [p['z'] for p in positions]
        valid = [sample for sample in samples if tracked(sample)]
        errors = sorted(float(t['stability']['center_err_deg']) for t in valid)
        truth_errors = sorted(truth_error_deg(t, odometry, args.mount_mode) for t in valid)
        truth_p95 = truth_errors[min(len(truth_errors) - 1, int(len(truth_errors) * 0.95))] if truth_errors else float('inf')
        ratio = len(valid) / len(samples) if samples else 0
        p95 = errors[min(len(errors) - 1, int(len(errors) * 0.95))] if errors else float('inf')
        result = dict(track_ratio=ratio, telemetry_samples=len(samples), valid_samples=len(valid),
                      p95_center_error_deg=p95, p95_truth_error_deg=truth_p95, max_center_error_deg=max(errors) if errors else None,
                      actual_distance_m=distance, height_range_m=max(heights) - min(heights),
                      odometry_samples=len(positions), sim_seconds=samples[-1]['ts_mono'] - samples[0]['ts_mono'])
        record('event', event='metrics', **result)
        assert distance >= args.speed * args.leg_seconds * 2, f'实际运动距离不足：{distance:.2f} m'
        assert max(heights) - min(heights) < 0.3, '小车高度异常，可能离开平台'
        assert ratio >= 0.95, f'TRACK 且 target.valid 比例不足：{ratio:.1%}'
        assert p95 <= 3.0, f'中心角误差 P95 超标：{p95:.2f}°'
        assert truth_p95 <= 3.0, f'车顶真值与跟踪框视线角误差超标：{truth_p95:.2f}°'
        record('event', event='pass')
    except BaseException as error:
        record('event', event='failure', error=str(error))
        raise
    finally:
        drive()
        send('stop')
        send('bye')
        sock.close()
        node.unsubscribe('/model/tracking_target/odometry')
        log.close()


if __name__ == '__main__':
    main()
