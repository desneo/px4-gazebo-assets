#!/usr/bin/env python3
"""运行独立中科演示，只管理本会话创建并记录身份的进程。"""
from __future__ import annotations
import argparse
import fcntl
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common.config import load_config
from scripts.session import (session_path, load_session, write_session,
                             record_process, process_matches, stop_session, edit_session)


def check_port(host, port, kind):
    family = socket.AF_INET6 if ':' in host else socket.AF_INET
    with socket.socket(family, kind) as probe:
        try:
            probe.bind((host, port))
        except OSError as exc:
            raise ValueError(f'中科端点已占用或不可用：{host}:{port}：{exc}') from exc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'parameters.ini'))
    parser.add_argument('--session', default='demo')
    gui = parser.add_mutually_exclusive_group()
    gui.add_argument('--gui', action='store_true')
    gui.add_argument('--headless', action='store_true')
    args = parser.parse_args(argv)
    children, logs = [], []
    closing = False
    initialized = False
    viewer_thread = None
    def request_stop(*_):
        nonlocal closing
        closing = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    try:
        config = load_config(args.config)
        manifest = session_path(args.session)
        manifest.parent.mkdir(parents=True, exist_ok=True)
        # 独占锁持续到所有本次子进程清理完成，拒绝同名重复启动。
        run_lock = manifest.with_suffix('.running.lock').open('a')
        try:
            fcntl.flock(run_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError('该中科会话正在运行，不能重复启动') from exc
        if manifest.exists():
            old = load_session(args.session)
            if any(process_matches(x) for x in old['processes'].values()):
                raise ValueError('清单仍有运行中的中科进程，请先停止该会话')
        net, video = config['network'], config['video']
        check_port(net['udp_bind'], net['udp_port'], socket.SOCK_DGRAM)
        check_port(video['rtsp_bind'], video['rtsp_port'], socket.SOCK_STREAM)
        library = ROOT / 'controller/build/lib/libgz-zhongke-d80-controller.so'
        if not library.exists():
            raise ValueError('尚未构建控制器，请先运行 bash scripts/build.sh')
        from tools.generate_model import generate
        generated = generate(config['config_path'], manifest.parent / 'generated')
        prefix = '/' + config['model']['name']
        partition = 'zhongke_d80_' + manifest.parent.name + '_' + uuid.uuid4().hex[:10]
        host = net['udp_bind']
        client_host = '127.0.0.1' if host == '0.0.0.0' else ('::1' if host == '::' else host)
        rtsp_host = video['rtsp_bind']
        if rtsp_host == '0.0.0.0':
            rtsp_host = '127.0.0.1'
        elif rtsp_host == '::':
            rtsp_host = '::1'
        if ':' in rtsp_host:
            rtsp_host = '[' + rtsp_host + ']'
        values = dict(state='starting', config_path=config['config_path'],
            config_hash=config['config_hash'], model=config['model']['name'],
            world=config['demo']['world_name'], aircraft_id=config['model']['aircraft_id'],
            gz_partition=partition, udp_host=client_host, udp_port=net['udp_port'],
            rtsp_port=video['rtsp_port'],
            rtsp_url=f"rtsp://{rtsp_host}:{video['rtsp_port']}{video['rtsp_mount']}",
            topic_prefix=prefix, image_topic=prefix + '/camera',
            diagnostics_topic=prefix + '/diagnostics',
            camera_info_topic=prefix + '/camera_info', world_path=str(generated / 'tests/world.sdf'),
            started_unix_ms=int(time.time() * 1000))
        # 已确认旧进程全部结束后建立新清单，避免保留旧进程身份。
        with edit_session(manifest) as p:
            for section in list(p.sections()):
                p.remove_section(section)
        write_session(args.session, values)
        initialized = True
        env = os.environ.copy()
        env['GZ_PARTITION'] = partition
        for key, entry in [('GZ_SIM_SYSTEM_PLUGIN_PATH', library.parent),
                           ('GZ_SIM_RESOURCE_PATH', ROOT.parent)]:
            env[key] = str(entry) + (os.pathsep + env[key] if env.get(key) else '')
        def spawn(key, command):
            log = (manifest.parent / (key + '.log')).open('w')
            logs.append(log)
            child = subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True,
                                     stdout=log, stderr=subprocess.STDOUT)
            children.append((key, child))
            record_process(args.session, key, child.pid)
        gz = ['gz', 'sim', '-r', '-v', '3']
        if not args.gui:
            gz += ['-s', '--headless-rendering']
        spawn('gazebo', gz + [values['world_path']])
        spawn('gateway', [sys.executable, '-u', str(ROOT / 'controller/service/gimbal_bridge.py'),
            '--config', config['config_path'], '--session', args.session,
            '--topic-prefix', prefix, '--image-topic', values['image_topic'],
            '--world', values['world'], '--model', values['model']])
        if args.gui:
            def configure_viewer():
                command=['gz','service','-s','/gui/camera/view_control/sensitivity',
                    '--reqtype','gz.msgs.Double','--reptype','gz.msgs.Boolean',
                    '--timeout','500','--req',f'data: {config["viewer"]["sensitivity"]}']
                deadline=time.monotonic()+20
                while not closing and time.monotonic()<deadline:
                    try:
                        result=subprocess.run(command,env=env,capture_output=True,text=True,timeout=2)
                    except subprocess.TimeoutExpired:
                        continue
                    if result.returncode==0 and 'data: true' in result.stdout:
                        print('中科浏览相机灵敏度已应用',flush=True)
                        return
                    time.sleep(.2)
                if not closing:
                    print('浏览相机灵敏度服务尚不可用，请按docs/mounting.md手动应用',flush=True)
            viewer_thread=threading.Thread(target=configure_viewer,daemon=True)
            viewer_thread.start()
        with edit_session(manifest) as p:
            p['session']['state'] = 'running'
        print(f'中科独立会话已启动：{args.session}，分区 {partition}', flush=True)
        print(f'清单：{manifest}\nUDP：{client_host}:{net["udp_port"]}\nRTSP：{values["rtsp_url"]}', flush=True)
        while not closing:
            if load_session(args.session).get('state') in ('stopping','stopped'):
                closing = True
                break
            failed = [(key, p.returncode) for key, p in children if p.poll() is not None]
            if failed:
                raise RuntimeError(f'中科子进程已退出：{failed}；请查看 {manifest.parent} 内日志')
            time.sleep(.2)
        return 0
    except (ValueError, OSError, RuntimeError) as exc:
        print(f'中科演示启动或运行失败：{exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        closing=True
        if viewer_thread:
            viewer_thread.join(timeout=3)
        if initialized:
            stop_session(args.session)
            for _, child in children:
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass
            with edit_session(session_path(args.session)) as p:
                p['session']['state'] = 'stopped'
        for log in logs:
            log.close()


if __name__ == '__main__':
    raise SystemExit(main())
