#!/usr/bin/env python3
"""本组件运行清单与进程身份管理，不接管其他会话。"""
from __future__ import annotations
import argparse
import configparser
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def session_path(name='demo') -> Path:
    p = Path(name)
    if p.suffix == '.ini' or p.is_absolute():
        result = p.expanduser().resolve()
        if not result.is_relative_to(ROOT / 'runs'):
            raise ValueError('会话清单必须位于本组件runs目录')
        return result
    if not re.fullmatch(r'[A-Za-z0-9_-]+', str(name)):
        raise ValueError('会话名只能包含英文、数字、下划线和连字符')
    return ROOT / 'runs' / str(name) / 'session.ini'


def start_ticks(pid: int) -> str:
    text = Path(f'/proc/{pid}/stat').read_text()
    return text[text.rfind(')') + 2:].split()[19]


@contextmanager
def edit_session(path):
    """跨进程串行修改清单，使用同目录唯一临时文件原子替换。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        parser = configparser.ConfigParser(interpolation=None)
        if path.exists():
            parser.read(path)
        yield parser
        with tempfile.NamedTemporaryFile('w', dir=path.parent, prefix='.session-',
                                         delete=False) as stream:
            temporary = Path(stream.name)
            stream.write('# 中科组件运行清单；停止前校验进程身份。\n')
            parser.write(stream)
        temporary.replace(path)


def load_session(name='demo') -> dict:
    path = session_path(name)
    p = configparser.ConfigParser(interpolation=None)
    with path.open() as stream:
        p.read_file(stream)
    out = dict(p['session'])
    for key in ('udp_port', 'rtsp_port', 'aircraft_id'):
        if key in out:
            out[key] = int(out[key])
    out['session_path'] = str(path)
    out['processes'] = {section[8:]: dict(p[section]) for section in p.sections()
                        if section.startswith('process ')}
    return out


read_session = load_session


def write_session(name, values: dict) -> Path:
    path = session_path(name)
    with edit_session(path) as p:
        p['session'] = {k: str(v) for k, v in values.items()
                       if k not in ('session_path', 'processes')}
    return path


def record_process(name, key: str, pid: int) -> dict:
    path = session_path(name)
    record = {'pid': str(pid), 'start_ticks': start_ticks(pid),
              'pgid': str(os.getpgid(pid))}
    with edit_session(path) as p:
        if not p.has_section('session'):
            raise ValueError('会话清单尚未由启动器建立')
        p['process ' + key] = record
    return record


def process_matches(record: dict) -> bool:
    try:
        pid = int(record['pid'])
        stat = Path(f'/proc/{pid}/stat').read_text()
        fields = stat[stat.rfind(')') + 2:].split()
        return (pid > 1 and fields[0] not in ('Z', 'X') and
                fields[19] == str(record['start_ticks']))
    except (OSError, ValueError, KeyError):
        return False


def stop_process(record: dict, timeout=3.0) -> bool:
    if not process_matches(record):
        return False
    pid = int(record['pid'])
    if pid == os.getpid():
        raise ValueError('停止工具不能停止自身')
    pgid = int(record.get('pgid', '0'))
    def send(sig):
        if process_matches(record):
            if pgid == pid and os.getpgid(pid) == pgid:
                os.killpg(pgid, sig)
            else:
                os.kill(pid, sig)
    try:
        send(signal.SIGTERM)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and process_matches(record):
            time.sleep(.05)
        if process_matches(record):
            send(signal.SIGKILL)
    except ProcessLookupError:
        pass
    return True


def stop_session(name='demo', only=None) -> int:
    try:
        session = load_session(name)
    except FileNotFoundError:
        print('会话未运行或清单不存在')
        return 0
    if only is None:
        with edit_session(session_path(name)) as p:
            p['session']['state'] = ('stopping' if any(process_matches(r)
                for r in session['processes'].values()) else 'stopped')
    count = 0
    for key, record in reversed(list(session['processes'].items())):
        if only is None or key == only:
            count += int(stop_process(record))
    if only is None and not any(process_matches(r) for r in session['processes'].values()):
        with edit_session(session_path(name)) as p:
            p['session']['state'] = 'stopped'
    print(f'已停止 {count} 个身份匹配的中科进程组')
    return count


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=('show', 'stop', 'record'))
    p.add_argument('--session', default='demo')
    p.add_argument('--only')
    p.add_argument('--key', default='ground_station')
    p.add_argument('--pid', type=int)
    a = p.parse_args()
    if a.action == 'show':
        print(json.dumps(load_session(a.session), ensure_ascii=False, indent=2))
    elif a.action == 'record':
        if not a.pid:
            p.error('record需要--pid')
        record_process(a.session, a.key, a.pid)
    else:
        stop_session(a.session, a.only)


if __name__ == '__main__':
    main()
