#!/usr/bin/env python3
"""向预览模型发送两轴速度扫掠，并打印机械角。"""
import argparse
import json
import math
import threading
import time

from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node


def main():
    parser = argparse.ArgumentParser(description='ASD-2-075-DD0 两轴扫掠预览')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--yaw-amp', type=float, default=40.0, help='yaw 速度幅值 °/s')
    parser.add_argument('--pitch-amp', type=float, default=15.0, help='pitch 速度幅值 °/s')
    parser.add_argument('--yaw-period', type=float, default=8.0, help='yaw 周期 s')
    parser.add_argument('--pitch-period', type=float, default=12.0, help='pitch 周期 s')
    parser.add_argument('--rate', type=float, default=20.0, help='指令发送频率 Hz')
    args = parser.parse_args()

    node = Node()
    pub = node.advertise('/aisheng_seeker/command', StringMsg)
    latest = {'yaw': 0.0, 'pitch': 0.0}
    lock = threading.Lock()

    def on_telemetry(msg):
        try:
            data = json.loads(msg.data)
            joints = data.get('joints_deg', {})
            with lock:
                latest['yaw'] = float(joints['yaw']['pos'])
                latest['pitch'] = float(joints['pitch']['pos'])
        except (ValueError, KeyError, TypeError):
            pass

    node.subscribe(StringMsg, '/aisheng_seeker/telemetry', on_telemetry)

    def send(cmd, **kwargs):
        payload = {'cmd': cmd, **kwargs}
        msg = StringMsg(data=json.dumps(payload, separators=(',', ':')))
        return pub.publish(msg)

    def stop():
        for _ in range(3):
            send('manual', yaw=0.0, pitch=0.0)
            time.sleep(0.05)

    print('开始两轴扫掠；Ctrl+C 停止。', flush=True)
    start = time.monotonic()
    next_print = start
    try:
        while True:
            now = time.monotonic()
            t = now - start
            if t >= args.duration:
                break
            yaw = args.yaw_amp * math.sin(2.0 * math.pi * t / args.yaw_period)
            pitch = args.pitch_amp * math.sin(2.0 * math.pi * t / args.pitch_period)
            send('manual', yaw=round(yaw, 3), pitch=round(pitch, 3))
            if now >= next_print:
                with lock:
                    print(f't={t:5.1f}s  cmd=({yaw:+6.1f},{pitch:+6.1f})°/s  '
                          f'mech=({latest["yaw"]:+7.2f},{latest["pitch"]:+7.2f})°',
                          flush=True)
                next_print = now + 0.5
            time.sleep(1.0 / max(1.0, args.rate))
    except KeyboardInterrupt:
        print('\n收到 Ctrl+C，停止。', flush=True)
    finally:
        stop()
        time.sleep(0.2)
        send('home')
        print('已发送停车和归中。', flush=True)


if __name__ == '__main__':
    main()
