#!/usr/bin/env python3
"""在线验证三轴独立复位；会转动吊舱，结束后停转，不恢复原视线。"""
import argparse
import json
from pathlib import Path
import socket
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--output', default='runs/axis_reset_validation/results.json')
    args = parser.parse_args()
    address = (socket.gethostbyname(args.host), args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 0))
    sock.settimeout(0.05)
    sequence = 0
    latest = None
    results = {}
    summary = {'axes': {}, 'passed': False}
    last_ping = 0.0

    def send(cmd, **fields):
        nonlocal sequence
        sequence += 1
        payload = json.dumps(dict(cmd=cmd, seq=sequence, **fields)).encode('utf-8')
        sock.sendto(payload, address)
        return sequence, payload

    def poll():
        nonlocal latest, last_ping
        if time.monotonic() - last_ping >= 1:
            send('ping')
            last_ping = time.monotonic()
        try:
            raw, source = sock.recvfrom(65535)
        except socket.timeout:
            return
        if source != address:
            return
        message = json.loads(raw)
        if message.get('type') == 'telemetry':
            latest = message
        elif message.get('type') == 'command_result':
            results[message.get('seq')] = message

    def wait(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            poll()
            if latest is not None and predicate(latest):
                return latest
        raise AssertionError('等待遥测条件超时')

    def reliable(cmd, **fields):
        seq, payload = send(cmd, **fields)
        deadline, retry = time.monotonic() + 3, time.monotonic() + 0.2
        while time.monotonic() < deadline and seq not in results:
            poll()
            if time.monotonic() >= retry:
                sock.sendto(payload, address)
                retry = time.monotonic() + 0.2
        assert seq in results, f'{cmd} 未收到回执'
        assert results[seq].get('ok'), results[seq]
        return results[seq]

    def positions(telemetry):
        assert telemetry['angle_convention'] == 'px4_frd'
        return {axis: telemetry['attitude_deg'][axis]['pos'] for axis in ('yaw', 'pitch', 'roll')}

    def delta(a, b, axis):
        return abs((a - b + 180) % 360 - 180) if axis == 'yaw' else abs(a - b)

    try:
        wait(lambda t: True, 5)
        summary['mount'] = latest.get('mount')
        reliable('stop')
        wait(lambda t: t['mode'] == 'IDLE' and all(abs(v) < 0.01 for v in t['cmd_dps'].values()))
        # 同时偏离三个显示零位，确保单轴复位不会把其他轴一起归零。
        deadline = time.monotonic() + 0.8
        next_manual = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_manual:
                send('manual', yaw=12, pitch=6, roll=4)
                next_manual = time.monotonic() + 0.1
            poll()
        reliable('stop')
        wait(lambda t: t['mode'] == 'IDLE' and all(abs(v) < 0.01 for v in t['cmd_dps'].values()))
        for axis in ('yaw', 'pitch', 'roll'):
            before = positions(latest)
            started = time.monotonic()
            ack = reliable('reset_axis', axis=axis)
            peak_hold_error = {other: 0.0 for other in before if other != axis}

            def arrived(t):
                current = positions(t)
                for other in peak_hold_error:
                    peak_hold_error[other] = max(peak_hold_error[other], delta(current[other], before[other], other))
                return t['mode'] == 'IDLE' and abs(current[axis]) < 0.65 and all(abs(v) < 0.01 for v in t['cmd_dps'].values())

            after = positions(wait(arrived))
            assert all(error < 0.8 for error in peak_hold_error.values()), peak_hold_error
            summary['axes'][axis] = dict(before=before, after=after, hold_error_deg=peak_hold_error,
                                          elapsed_s=time.monotonic() - started, ack=ack)
            print(f'PASS {axis}: {before[axis]:.3f}° → {after[axis]:.3f}°, other axes {peak_hold_error}', flush=True)
        summary['passed'] = True
    finally:
        try:
            send('manual', yaw=0, pitch=0, roll=0)
            send('stop')
            send('bye')
        finally:
            sock.close()
            output = Path(args.output)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
