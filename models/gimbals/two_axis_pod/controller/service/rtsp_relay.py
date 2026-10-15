"""同机多吊舱共享的 RTSP 中继；本地 Unix socket 注册带租约的视频路径。"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import socket
import socketserver
import subprocess
import sys
import tempfile
import threading
import time
import uuid

LEASE_SECONDS = 10.0
IDLE_SECONDS = 30.0
RTSP_LAUNCH = (
    '( udpsrc address=127.0.0.1 port={udp_port} '
    'caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" '
    '! queue max-size-buffers=1000 max-size-time=0 max-size-bytes=0 '
    '! rtph264depay '
    '! queue leaky=downstream max-size-buffers=3 max-size-time=0 max-size-bytes=0 '
    '! rtph264pay name=pay0 pt=96 config-interval=1 )'
)


def aircraft_mount(base: str, aircraft_id: int) -> str:
    if isinstance(aircraft_id, bool) or not isinstance(aircraft_id, int) or aircraft_id < 0:
        raise ValueError('aircraft_id 必须为非负整数')
    base = '/' + base.strip('/')
    if not re.fullmatch(r'/[A-Za-z0-9_/-]+', base):
        raise ValueError('RTSP 路径前缀只允许字母、数字、下划线、短横线和斜线')
    return f'{base}_{aircraft_id}'


def runtime_dir() -> Path:
    path = Path(tempfile.gettempdir()) / f'two-axis-rtsp-{os.getuid()}'
    path.mkdir(mode=0o700, exist_ok=True)
    if path.is_symlink() or path.stat().st_uid != os.getuid() or path.stat().st_mode & 0o077:
        raise RuntimeError(f'共享 RTSP 运行目录权限不安全: {path}')
    return path


def request(port: int, payload: dict) -> dict:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(4)
        client.connect(str(runtime_dir() / f'{port}.sock'))
        client.sendall(json.dumps(payload).encode() + b'\n')
        with client.makefile('rb') as stream:
            response = json.loads(stream.readline(8192))
    if not response.get('ok'):
        raise RuntimeError(response.get('error', 'RTSP 中继请求失败'))
    return response


def ensure_relay(port: int):
    try:
        request(port, {'op': 'ping'})
        return
    except (OSError, ValueError, RuntimeError):
        pass
    # 多网关同时启动时，中继进程用 flock 保证每端口只有一个所有者。
    with (runtime_dir() / f'{port}.log').open('ab') as log:
        process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--port', str(port)],
                         stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                         start_new_session=True, close_fds=True)
    threading.Thread(target=process.wait, daemon=True).start()
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        try:
            request(port, {'op': 'ping'})
            return
        except (OSError, ValueError, RuntimeError):
            time.sleep(.1)
    raise RuntimeError(f'共享 RTSP 中继启动失败，查看 {runtime_dir() / f"{port}.log"}')


class RelayRegistration:
    def __init__(self, port: int, mount: str, udp_port: int):
        self.port, self.mount, self.udp_port = port, mount, udp_port
        self.token = uuid.uuid4().hex
        self.clients = 0
        self._stop = threading.Event()
        ensure_relay(port)
        self._register()
        self._thread = threading.Thread(target=self._heartbeat, daemon=True)
        self._thread.start()

    def _register(self):
        request(self.port, dict(op='register', mount=self.mount,
                                udp_port=self.udp_port, token=self.token))

    def _heartbeat(self):
        failed = False
        while not self._stop.wait(2):
            try:
                response = request(self.port, dict(op='renew', mount=self.mount, token=self.token))
                self.clients = response['clients']
                failed = False
            except (OSError, ValueError, RuntimeError) as exc:
                self.clients = 0
                if not failed:
                    print(f'[video] RTSP 租约续期失败: {exc}；尝试重新注册', flush=True)
                failed = True
                if self._stop.is_set():
                    break
                try:
                    ensure_relay(self.port)
                    self._register()
                except (OSError, ValueError, RuntimeError):
                    pass

    def close(self):
        self._stop.set()
        self._thread.join(timeout=5)
        try:
            request(self.port, dict(op='remove', mount=self.mount, token=self.token))
        except (OSError, ValueError, RuntimeError):
            pass  # 崩溃/不可达时由租约超时清理。


def serve(port: int):
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstRtspServer', '1.0')
    from gi.repository import GLib, Gst, GstRtspServer
    Gst.init(None)
    directory = runtime_dir()
    lock = (directory / f'{port}.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return
    rtsp = GstRtspServer.RTSPServer()
    rtsp.set_service(str(port))
    source_id = rtsp.attach(None)
    if not source_id:
        raise RuntimeError(f'RTSP 端口 {port} 已被其他服务占用')
    mounts = rtsp.get_mount_points()
    streams = {}
    last_used = time.monotonic()
    loop = GLib.MainLoop()

    def remove(path):
        mounts.remove_factory(path)
        del streams[path]

    def dispatch(data):
        nonlocal last_used
        op = data.get('op')
        if op == 'ping':
            last_used = time.monotonic()
            return dict(ok=True, pid=os.getpid(), paths=sorted(streams))
        path, token = data.get('mount'), data.get('token')
        if not isinstance(path, str) or not re.fullmatch(r'/[A-Za-z0-9_/-]+', path):
            raise ValueError('非法 RTSP 路径')
        if not isinstance(token, str) or not token:
            raise ValueError('缺少注册令牌')
        now = time.monotonic()
        current = streams.get(path)
        if current and now - current['seen'] > LEASE_SECONDS:
            remove(path)
            current = None
        if current and current['token'] != token:
            raise ValueError(f'RTSP 路径 {path} 已注册；请使用不同的 aircraft_id')
        if op == 'register':
            udp_port = data.get('udp_port')
            if type(udp_port) is not int or not 1 <= udp_port <= 65535:
                raise ValueError('非法内部 RTP 端口')
            if any(s['port'] == udp_port and p != path for p, s in streams.items()):
                raise ValueError(f'内部 RTP 端口 {udp_port} 已被使用')
            if current:
                if current['port'] != udp_port:
                    raise ValueError('已注册路径的 RTP 端口不能更改')
                current['seen'] = now
            else:
                factory = GstRtspServer.RTSPMediaFactory()
                factory.set_launch(RTSP_LAUNCH.format(udp_port=udp_port))
                factory.set_shared(True)
                state = dict(token=token, port=udp_port, seen=now, clients=0)
                def configured(_factory, media):
                    state['clients'] += 1
                    def unprepared(_media):
                        state['clients'] = max(0, state['clients'] - 1)
                    media.connect('unprepared', unprepared)
                factory.connect('media-configure', configured)
                streams[path] = state
                mounts.add_factory(path, factory)
                print(f'[relay] 注册 :{port}{path} <- RTP {udp_port}', flush=True)
        elif op == 'renew':
            if current is None:
                raise ValueError('注册已过期或中继已重启')
            current['seen'] = now
        elif op == 'remove':
            if current:
                remove(path)
        else:
            raise ValueError('未知操作')
        last_used = now
        return dict(ok=True, clients=streams.get(path, {}).get('clients', 0))

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.request.settimeout(4)
            done = threading.Event()
            result = {}
            try:
                data = json.loads(self.rfile.readline(8192))
                if not isinstance(data, dict):
                    raise ValueError('请求必须为 JSON 对象')
                def on_main():
                    try:
                        result.update(dispatch(data))
                    except Exception as exc:
                        result.update(ok=False, error=str(exc))
                    finally:
                        done.set()
                    return False
                GLib.idle_add(on_main)
                if not done.wait(3):
                    raise TimeoutError('中继操作超时')
            except Exception as exc:
                result = dict(ok=False, error=str(exc))
            self.wfile.write(json.dumps(result).encode() + b'\n')

    sock_path = directory / f'{port}.sock'
    sock_path.unlink(missing_ok=True)
    with socketserver.ThreadingUnixStreamServer(str(sock_path), Handler) as control:
        control.daemon_threads = True
        thread = threading.Thread(target=control.serve_forever, daemon=True)
        thread.start()
        def expire():
            now = time.monotonic()
            for path, state in list(streams.items()):
                if now - state['seen'] > LEASE_SECONDS:
                    remove(path)
            if not streams and now - last_used > IDLE_SECONDS:
                loop.quit()
                return False
            return True
        GLib.timeout_add_seconds(1, expire)
        print(f'[relay] 共享 RTSP 服务监听 {port}', flush=True)
        try:
            loop.run()
        finally:
            control.shutdown()
            sock_path.unlink(missing_ok=True)
            GLib.source_remove(source_id)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='两轴吊舱共享 RTSP 中继（网关自动启动）')
    parser.add_argument('--port', type=int, default=8555)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('port 必须为 1～65535')
    serve(args.port)
