#!/usr/bin/env python3
"""固定棋盘世界的在线控制验证；运行时会转动吊舱。"""
import argparse
import json
import socket
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    address = (socket.gethostbyname(args.host), args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 0))
    sock.settimeout(0.2)
    sequence = 0

    def send(command, **parameters):
        nonlocal sequence
        sequence += 1
        payload = dict(cmd=command, seq=sequence, **parameters)
        sock.sendto(json.dumps(payload).encode(), address)

    def collect(duration, predicate=None):
        deadline = time.monotonic() + duration
        latest, results, next_ping = None, [], 0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_ping:
                send('ping')
                next_ping = time.monotonic() + 1
            try:
                payload, source = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if source != address:
                continue
            data = json.loads(payload)
            if data.get('type') == 'command_result':
                results.append(data)
            if data.get('type') == 'telemetry':
                latest = data
                if predicate and predicate(latest):
                    break
        if latest is None:
            raise RuntimeError('未收到遥测')
        return latest, results

    try:
        send('reset')
        telemetry, results = collect(15, lambda t:
            t['mode'] == 'IDLE' and abs(t['attitude_deg']['pitch']['pos']) < 1)
        print('reset', results, telemetry['joints_deg'], flush=True)
        assert telemetry['mode'] == 'IDLE', 'reset 未在15秒内到位'
        assert telemetry['angle_convention'] == 'px4_frd'
        assert all(abs(telemetry['attitude_deg'][axis]['pos']) < 1 for axis in ('yaw', 'pitch', 'roll'))

        send('track', box=[0.45, 0.8, 0.08, 0.08])
        telemetry, results = collect(3)
        print('track', results, telemetry['tracking'], telemetry['target'], flush=True)
        assert any(r.get('cmd') == 'track' and r.get('ok') for r in results)
        assert telemetry['tracking']['state'] == 'TRACK' and telemetry['target']['valid']

        send('stop')
        telemetry, results = collect(1)
        print('stop', results, telemetry['cmd_dps'], flush=True)
        assert telemetry['mode'] == 'IDLE'
        assert all(v == 0 for v in telemetry['cmd_dps'].values())

        send('home')
        telemetry, results = collect(15, lambda t:
            t['mode'] == 'IDLE' and abs(t['attitude_deg']['pitch']['pos'] - 90) < 1)
        print('home', results, telemetry['joints_deg'], flush=True)
        assert telemetry['mode'] == 'IDLE', 'home 未在15秒内到位'
        # 天顶处 yaw/roll 的欧拉分解不唯一，不把显示分量作为独立到位条件。
        assert telemetry['attitude_deg']['singular']
        print('PASS live reset/track/stop/home', flush=True)
    finally:
        try:
            send('manual', yaw=0, pitch=0, roll=0)
            send('bye')
        finally:
            sock.close()


if __name__ == '__main__':
    main()
