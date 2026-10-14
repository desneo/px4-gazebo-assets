#!/usr/bin/env python3
"""用 PID 与内核启动时间识别本次场景进程，避免误停复用 PID 的进程。"""

import argparse
import json
import os
from pathlib import Path
import signal
import time


def process_info(pid):
    try:
        # /proc/stat 的进程名允许空格与括号，字段从最后一个右括号后解析。
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return {'state': fields[0], 'group': int(fields[2]),
                'session': int(fields[3]), 'start_time': fields[19]}
    except (OSError, IndexError):
        return None


def group_members(state):
    """组长退出后仍清理同一 session 的子进程，避免遗留渲染子进程。"""
    pid = state['pid']
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        info = process_info(path.name)
        if (info and info['state'] != 'Z' and info['group'] == pid
                and info['session'] == pid and int(info['start_time']) >= int(state['start_time'])):
            return True
    return False


def read_live(path):
    try:
        state = json.loads(path.read_text())
        pid = state['pid']
        if type(pid) is not int or pid <= 1:
            return None
        info = process_info(pid)
        if info and info['start_time'] != state['start_time']:
            return None
        if state['group']:
            if info and info['state'] != 'Z' and info['group'] == pid and info['session'] == pid:
                return state
            return state if group_members(state) else None
        if info is None or info['state'] == 'Z':
            return None
        return state
    except (OSError, KeyError, ValueError, TypeError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('record', 'live', 'signal'))
    parser.add_argument('path', type=Path)
    parser.add_argument('value', nargs='?')
    parser.add_argument('--launcher', action='store_true')
    args = parser.parse_args()
    if args.action == 'record':
        pid = int(args.value)
        deadline = time.monotonic() + 1
        while True:
            info = process_info(pid)
            if info is None or info['state'] == 'Z':
                return 1
            if args.launcher or (info['group'] == pid and info['session'] == pid):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('服务必须由 setsid 启动为独立进程组')
            time.sleep(.01)
        if pid <= 1:
            return 1
        temporary = args.path.with_suffix(args.path.suffix + '.tmp')
        temporary.write_text(json.dumps({'pid': pid, 'start_time': info['start_time'],
                                        'group': not args.launcher}) + '\n')
        temporary.replace(args.path)
        return 0
    state = read_live(args.path)
    if state is None:
        return 1
    if args.action == 'signal':
        signum = getattr(signal, 'SIG' + args.value)
        try:
            if state['group']:
                os.killpg(state['pid'], signum)
            else:
                os.kill(state['pid'], signum)
        except ProcessLookupError:
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
