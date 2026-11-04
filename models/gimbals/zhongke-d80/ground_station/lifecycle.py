"""管理本客户端进程；默认独立连接，显式session才关联仿真清单。"""
import fcntl
import configparser
import os
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from ground_station.configuration import parse_arguments,resolve_configuration,client_manifest_path
from scripts.session import load_session,process_matches,record_process,stop_process,stop_session,write_session


def is_ground_station_process(record):
    """清单启动身份和真实程序路径同时匹配，避免误停其他模型客户端。"""
    if not process_matches(record):
        return False
    try:
        command=[os.fsdecode(value) for value in
                 Path(f'/proc/{int(record["pid"])}/cmdline').read_bytes().split(b'\0')]
        if not re.fullmatch(r'python(?:\d+(?:\.\d+)*)?',Path(command[0]).name):
            return False
        index=1
        while index<len(command) and command[index] in ('-u','-B','-E','-s','-S','-I','-O','-OO','-q'):
            index+=1
        script=Path(command[index])
        return script.is_absolute() and script.resolve()==ROOT/'ground_station/ground_station.py'
    except (OSError,ValueError,KeyError,IndexError):
        return False


def stop_running_ground_stations():
    """无参数只停止本组件清单登记的客户端，不停止其场景或进程组。"""
    stopped=0
    seen=set()
    for path in sorted((ROOT/'runs').glob('*/session.ini')):
        try:
            session=load_session(path)
        except (OSError,ValueError,KeyError,configparser.Error):
            continue
        record=session['processes'].get('ground_station')
        if not record or not is_ground_station_process(record):
            continue
        identity=(str(record['pid']),str(record['start_ticks']))
        if identity in seen:
            continue
        seen.add(identity)
        # 地面站使用线程；仅向此PID发信号，不波及同组的场景服务。
        stopped+=int(stop_process({**record,'pgid':'0'}))
    if stopped:
        print(f'已停止 {stopped} 个身份匹配的中科地面站')
    else:
        print('当前没有运行中的中科地面站')
    return stopped


def main(argv=None):
    argv=list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in ('run','stop'):
        print('地面站进程入口需要run或stop动作',file=sys.stderr)
        return 1
    action=argv.pop(0)
    args=parse_arguments(argv)
    lock=None
    try:
        if action=='stop':
            if not argv:
                stop_running_ground_stations()
                return 0
            target=args.session or str(client_manifest_path(args.station_config))
            stop_session(target,only='ground_station')
            return 0
        cfg,preferences,session=resolve_configuration(args)
        manifest=(Path(session['session_path']) if session else
                  client_manifest_path(args.station_config))
        manifest.parent.mkdir(parents=True,exist_ok=True)
        lock=manifest.with_suffix('.ground-station.lock').open('a')
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError('对应配置的地面站已运行，请先关闭或停止') from exc
        previous=(session or (load_session(manifest) if manifest.exists() else {'processes':{}}))
        owned=previous['processes'].get('ground_station')
        if owned and process_matches(owned):
            raise ValueError('对应配置的地面站已有身份匹配进程，不接管该客户端')
        if not session:
            write_session(manifest,dict(state='running',client_mode='independent',
                station_config=args.station_config,config_path=args.config,config_hash=cfg['config_hash'],
                udp_host=preferences['udp_host'],udp_port=preferences['udp_port'],
                local_bind=preferences['local_bind'],local_port=preferences['local_port'],
                rtsp_url=preferences['rtsp_url']))
        if os.getpgrp()!=os.getpid():os.setsid()
        record_process(manifest,'ground_station',os.getpid())
        os.set_inheritable(lock.fileno(),True)
        os.environ['PYTHONFAULTHANDLER']='1'
        command=[sys.executable,str(ROOT/'ground_station/ground_station.py'),
                 '--station-config',args.station_config]
        if args.session:command+=['--session',args.session]
        elif args.config:command+=['--config',args.config]
        for name in ('udp_host','udp_port','rtsp_url','window_title'):
            value=getattr(args,name,None)
            if value is not None:command+=['--'+name.replace('_','-'),str(value)]
        if not args.save_preferences:
            # 保留CLI临时连接语义，避免子进程把默认路径误认作显式保存授权。
            index=command.index('--station-config')
            del command[index:index+2]
        if args.duration:command+=['--duration',str(args.duration)]
        if args.screenshot:command+=['--snapshot',args.screenshot]
        if args.selftest:command+=['--selftest']
        if args.decode_test:command+=['--decode-test']
        os.execv(sys.executable,command)
    except (OSError,ValueError,KeyError) as exc:
        print(f'中科地面站进程管理失败：{exc}',file=sys.stderr)
        return 1
    finally:
        # 真正exec成功时此进程被替换；失败或测试替身返回时及时关闭FD。
        if lock is not None:lock.close()


if __name__=='__main__':
    raise SystemExit(main())
