#!/usr/bin/env python3
"""读取中科会话清单发送标准二进制命令；发送成功不代表执行确认。"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import socket
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common import protocol
from scripts.session import load_session

COMMANDS = dict(nop=protocol.CMD_NOP, manual=protocol.CMD_MANUAL,
    angles=protocol.CMD_FRAME_ANGLE, home=protocol.CMD_HOME,
    point=protocol.CMD_POINT_TRACK, auto=protocol.CMD_AUTO_LOCK,
    stop=protocol.CMD_STOP_TRACK)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=tuple(COMMANDS))
    p.add_argument('--session', default='demo')
    p.add_argument('--x', type=float, default=0, help='manual: °/s；angles: °；point: 中心原点像素')
    p.add_argument('--y', type=float, default=0, help='manual: °/s；angles: °；point: 中心原点像素')
    p.add_argument('--duration', type=float, default=1, help='接收真实遥测时长；manual每0.1s刷新')
    p.add_argument('--quiet', action='store_true', help='仅打印最终接收统计')
    args = p.parse_args(argv)
    if not 0 < args.duration <= 3600:
        p.error('duration必须在0至3600秒之间')
    try:
        session = load_session(args.session)
        destination = (session['udp_host'], int(session['udp_port']))
        cmd = COMMANDS[args.command]
        unit = .1 if args.command == 'manual' else .01 if args.command == 'angles' else 1
        x = protocol.quantize(args.x, unit, -32768, 32767, 'X')
        y = protocol.quantize(args.y, unit, -32768, 32767, 'Y')
        raw = protocol.encode_command(cmd, x, y)
        family = socket.AF_INET6 if ':' in destination[0] else socket.AF_INET
        count, first, last = 0, None, None
        with socket.socket(family, socket.SOCK_DGRAM) as client:
            client.settimeout(.05)
            client.connect(destination)
            client.send(raw)
            if args.command == 'stop':
                client.send(protocol.encode_command(protocol.CMD_MANUAL))
            print(f'已发送中科指令0x{cmd:02X}至{destination[0]}:{destination[1]}；无厂商ACK')
            started = time.monotonic()
            next_send = started + (.1 if args.command == 'manual' else .2)
            try:
                while time.monotonic() - started < args.duration:
                    now = time.monotonic()
                    if now >= next_send:
                        client.send(raw if args.command == 'manual' else
                                    protocol.encode_command(protocol.CMD_NOP))
                        next_send = now + (.1 if args.command == 'manual' else .2)
                    try:
                        data = client.recv(2048)
                    except socket.timeout:
                        continue
                    frame = protocol.decode_telemetry(data)
                    count += 1
                    first = time.monotonic() if first is None else first
                    last = time.monotonic()
                    if not args.quiet:
                        print(json.dumps(frame, ensure_ascii=False, allow_nan=False))
            finally:
                if args.command == 'manual':
                    client.send(protocol.encode_command(protocol.CMD_MANUAL))
        rate = (count - 1) / (last - first) if count > 1 and last > first else 0
        print(f'有效32字节遥测：{count}帧，采样区间频率{rate:.2f}Hz')
        return 0 if count else 1
    except (OSError, ValueError, KeyError) as exc:
        print(f'中科命令工具失败：{exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
