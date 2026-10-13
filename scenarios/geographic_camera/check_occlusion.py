#!/usr/bin/env python3
"""在地理场景中验证两轮 2 秒真实实体遮挡与自动重捕获。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import socket
import time

from control_scene import set_pose
from scene_geometry import initialize_camera, panel_position


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only', action='store_true', help='只初始化场景相机下视角度')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--box', type=float, nargs=4, default=[0.465, 0.376, 0.072, 0.241])
    parser.add_argument('--output', type=Path, default=Path('runs/geographic_camera/occlusion.jsonl'))
    parser.add_argument('--mount-mode', choices=['skyward', 'downward', 'sideward'], default='skyward')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    address = (socket.gethostbyname(args.host), args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 0))
    sock.settimeout(0.05)
    sequence = 0
    next_ping = 0
    log = args.output.open('w', encoding='utf-8')
    pool = ThreadPoolExecutor(max_workers=1)

    def record(kind, **data):
        entry = dict(wall_time=time.time(), monotonic=time.monotonic(), kind=kind, **data)
        log.write(json.dumps(entry, ensure_ascii=False) + '\n')
        log.flush()
        if kind == 'event':
            print(data, flush=True)

    def send(command, **parameters):
        nonlocal sequence
        sequence += 1
        payload = dict(cmd=command, seq=sequence, **parameters)
        sock.sendto(json.dumps(payload).encode(), address)
        record('command', payload=payload)

    def collect(duration, predicate=None):
        nonlocal next_ping
        deadline = time.monotonic() + duration
        samples = []
        while time.monotonic() < deadline:
            if time.monotonic() >= next_ping:
                send('ping')
                next_ping = time.monotonic() + 0.5
            try:
                payload, source = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if source != address:
                continue
            data = json.loads(payload)
            record('received', payload=data)
            if data.get('type') == 'telemetry':
                samples.append(data)
                if predicate and predicate(data):
                    return samples
        return samples

    def tracked(t):
        return t.get('tracking', {}).get('state') == 'TRACK' and t.get('target', {}).get('valid')

    try:
        record('event', event='start', client_port=sock.getsockname()[1], box=args.box)
        available = collect(30, lambda t: True)
        assert available, '初始化前未收到遥测'
        set_pose('occlusion_panel', 40, 0, 40)
        initialize_camera(send, collect, args.mount_mode)
        record('event', event='prepared')
        if args.prepare_only:
            print('场景相机已复位至下视目标', flush=True)
            return
        send('track', box=args.box)
        initial = collect(5, tracked)
        assert initial and tracked(initial[-1]), '未进入 TRACK，请确认目标位置及框选范围'
        collect(1)
        for cycle in (1, 2):
            # 使用后台服务调用，同时保持遥测接收和心跳，避免客户端 watchdog 干扰。
            future = pool.submit(set_pose, 'occlusion_panel', *panel_position(args.mount_mode))
            samples = []
            while not future.done():
                samples += collect(0.1)
            future.result()
            record('event', event='occlusion_confirmed', cycle=cycle)
            samples += collect(2.0)
            future = pool.submit(set_pose, 'occlusion_panel', 40, 0, 40)
            while not future.done():
                samples += collect(0.1)
            future.result()
            record('event', event='clear_confirmed', cycle=cycle)
            states = [t.get('tracking', {}).get('state') for t in samples]
            recovered = collect(4, tracked)
            all_states = states + [t.get('tracking', {}).get('state') for t in recovered]
            assert samples, '遮挡期间未收到遥测'
            assert 'TEMP_LOST' in states, f'第 {cycle} 轮未观察到 TEMP_LOST，states={set(states)}'
            assert 'LOST' not in all_states, f'第 {cycle} 轮进入了终止 LOST'
            assert recovered and tracked(recovered[-1]), f'第 {cycle} 轮未自动恢复 TRACK'
            record('event', event='cycle_pass', cycle=cycle, states=sorted(set(all_states)))
            stable = collect(1)
            assert stable and all(tracked(t) for t in stable), f'第 {cycle} 轮恢复后 TRACK 不稳定'
        record('event', event='pass', cycles=2)
        print(f'PASS：两轮 2 秒实体遮挡重捕获，证据 {args.output.resolve()}', flush=True)
    except BaseException as error:
        record('event', event='failure', error=str(error))
        raise
    finally:
        # 等待在途 set_pose，再恢复遮挡板，避免后台调用覆盖清理结果。
        pool.shutdown(wait=True)
        try:
            set_pose('occlusion_panel', 40, 0, 40)
        finally:
            send('bye')
            sock.close()
            log.close()


if __name__ == '__main__':
    import os
    os.environ.setdefault('GZ_PARTITION', 'geographic_camera')
    main()
