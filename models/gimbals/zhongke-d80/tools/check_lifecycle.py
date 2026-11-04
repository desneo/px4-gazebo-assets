#!/usr/bin/env python3
"""真实检查重复启动、地面站定向停止及故障启动回收；不停止受验现有世界。"""
from __future__ import annotations

import argparse
import configparser
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.session import load_session,process_matches,start_ticks,stop_session


def run_logged(command,path,*,timeout=15,env=None):
    with path.open('w') as stream:
        result=subprocess.run(command,cwd=ROOT,env=env,stdout=stream,
                              stderr=subprocess.STDOUT,timeout=timeout,start_new_session=True)
    return result.returncode,path.read_text(errors='replace')


def protected_services(session):
    services={key:session['processes'][key] for key in ('gazebo','gateway')}
    if not all(process_matches(value) for value in services.values()):
        raise AssertionError('受验会话的Gazebo或网关未运行')
    return services


def assert_protected(session,records):
    current=load_session(session['session_path'])
    assert all(current['processes'].get(k)==v and process_matches(v)
               for k,v in records.items()),'现有世界/网关身份被改变或停止'
    host=session['udp_host']
    family=socket.AF_INET6 if ':' in host else socket.AF_INET
    with socket.socket(family,socket.SOCK_DGRAM) as probe:
        try:probe.bind((host,session['udp_port']))
        except OSError:pass
        else:raise AssertionError('现有UDP监听端点已释放')
    with socket.create_connection((host,session['rtsp_port']),timeout=1):
        pass


def check_live(name,output):
    session=load_session(name)
    records=protected_services(session)
    assert_protected(session,records)
    old_station=session['processes'].get('ground_station')
    if old_station and process_matches(old_station):
        raise AssertionError('现有地面站仍在运行；不能接管或停止其他操作者的地面站')
    code,log=run_logged(['bash',str(ROOT/'scripts/run_demo.sh'),'--session',name,
        '--config',session['config_path'],'--headless'],output/'duplicate.log',timeout=8)
    assert code!=0 and ('正在运行' in log or '仍有运行' in log),'同名运行会话未明确拒绝重复启动'
    assert_protected(session,records)
    code,log=run_logged(['bash',str(ROOT/'ground_station/run.sh'),'--session',name,
        '--decode-test','--duration','2'],output/'station_normal.log',timeout=12)
    assert code==0,f'地面站2秒真实解码未正常退出：{log}'
    assert_protected(session,records)
    station=None
    with (output/'station_stop.log').open('w') as stream:
        try:
            station=subprocess.Popen(['bash',str(ROOT/'ground_station/run.sh'),
                '--session',name,'--decode-test','--duration','30'],cwd=ROOT,
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+4
            owned=None
            while time.monotonic()<deadline:
                record=load_session(name)['processes'].get('ground_station',{})
                if record.get('pid')==str(station.pid) and process_matches(record):
                    owned=record;break
                if station.poll() is not None:
                    raise AssertionError('受控地面站在登记前退出')
                time.sleep(.02)
            assert owned,'受控地面站未按自身PID登记'
            code,log=run_logged(['bash',str(ROOT/'ground_station/stop.sh'),'--session',name],
                                output/'station_stop_command.log',timeout=6)
            assert code==0,f'定向停止地面站失败：{log}'
            station.wait(timeout=5)
            assert not process_matches(owned),'地面站身份仍存活'
        finally:
            # 仅回收本工具Popen创建的地面站，绝不stop受验世界。
            if station is not None and station.poll() is None:
                station.terminate();station.wait(timeout=5)
    assert_protected(session,records)
    return {'passed':True,'duplicate_start_rejected':True,
            'station_decode_two_seconds_exit':True,'station_only_stop':True,
            'protected_service_pids':{k:int(v['pid']) for k,v in records.items()}},(session,records)


def free_port(kind,forbidden=()):
    for _ in range(10):
        with socket.socket(socket.AF_INET,kind) as candidate:
            candidate.bind(('127.0.0.1',0))
            value=candidate.getsockname()[1]
        if value not in forbidden:return value
    raise AssertionError('不能分配独立测试端口')


def check_failure(output,protected=None):
    identifier=uuid.uuid4().hex[:10]
    fault_dir=output/('fault_'+identifier)
    fault_dir.mkdir()
    fault_manifest=fault_dir/'session.ini'
    fake_bin=fault_dir/'bin';fake_bin.mkdir()
    fake_gz=fake_bin/'gz'
    fake_gz.write_text('#!/bin/sh\n# 仅本测试子进程PATH使用，模拟Gazebo立即退出。\nexit 42\n')
    fake_gz.chmod(0o755)
    parser=configparser.ConfigParser(interpolation=None)
    parser.read(ROOT/'parameters.ini')
    udp=free_port(socket.SOCK_DGRAM)
    rtsp=free_port(socket.SOCK_STREAM,forbidden=(udp,))
    parser['network']['udp_bind']='127.0.0.1';parser['network']['udp_port']=str(udp)
    parser['video']['rtsp_bind']='127.0.0.1';parser['video']['rtsp_port']=str(rtsp)
    parser['video']['rtsp_mount']='/zhongke_d80_fault_'+identifier
    parser['model']['name']='zhongke_d80_fault_'+identifier
    parser['demo']['world_name']='zhongke_d80_fault_'+identifier
    config=fault_dir/'parameters.ini'
    with config.open('w') as stream:
        stream.write('# 生命周期故障夹具的独立运行配置，不修改人工性能INI。\n')
        parser.write(stream)
    env=os.environ.copy()
    env['PATH']=str(fake_bin)+os.pathsep+env.get('PATH','')
    parent={'pid':str(os.getppid()),'start_ticks':start_ticks(os.getppid())}
    # 与故障运行端点无关的已占用socket，确认清理不会释放其他资源。
    guard=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    guard.bind(('127.0.0.1',0));guard.settimeout(1)
    started=False
    try:
        started=True
        code,log=run_logged([sys.executable,str(ROOT/'scripts/runtime.py'),
            '--config',str(config),'--session',str(fault_manifest),'--headless'],
            fault_dir/'runtime.log',timeout=15,env=env)
        assert code!=0 and '42' in log and 'gazebo' in log,'未观察到Gazebo退出42的启动失败'
        session=load_session(fault_manifest)
        assert 'gateway' in session['processes'] and 'gazebo' in session['processes'],'失败运行没有记录自有子进程'
        alive=[key for key,value in session['processes'].items() if process_matches(value)]
        assert not alive,f'runtime失败返回后仍残留自有子进程：{alive}'
        assert session['state']=='stopped','失败运行没有进入stopped状态'
        for kind,port in ((socket.SOCK_DGRAM,udp),(socket.SOCK_STREAM,rtsp)):
            with socket.socket(socket.AF_INET,kind) as probe:
                probe.bind(('127.0.0.1',port))
        guard.sendto(b'untouched',guard.getsockname())
        assert guard.recv(64)==b'untouched','独立占用端点被回收动作影响'
        assert process_matches(parent),'其他既有父进程被回收动作影响'
        if protected:assert_protected(*protected)
        first=stop_session(fault_manifest)
        second=stop_session(fault_manifest)
        assert first==second==0,'失败回收后重复stop不幂等或仍有残留进程'
        return dict(passed=True,gazebo_exit_code=42,runtime_exit_code=code,
                    owned_children_reclaimed=True,repeated_stop_counts=[first,second],
                    udp_port_released=udp,rtsp_port_released=rtsp,
                    unrelated_parent_survived=int(parent['pid']),
                    unrelated_bound_endpoint_survived=True,
                    protected_world_preserved=bool(protected),session=str(fault_manifest))
    finally:
        guard.close()
        # 清理范围只含本工具创建的故障清单；从不stop受验validation。
        if started and fault_manifest.exists():stop_session(fault_manifest)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',default='validation',help='保留运行、不被停止的现有会话')
    parser.add_argument('--failure-only',action='store_true',help='仅运行独立故障会话，不使用现有世界/GS登记')
    parser.add_argument('--live-only',action='store_true',help='只运行需要现有世界的检查')
    args=parser.parse_args(argv)
    if args.failure_only and args.live_only:parser.error('两种独立运行选项不能同时使用')
    output=ROOT/'runs/lifecycle'/time.strftime('%Y%m%d-%H%M%S')
    output.mkdir(parents=True,exist_ok=True)
    result={'live':None,'failure':None,'mouse_interaction':'未验证真实鼠标/无DISPLAY交互'}
    try:
        protected=None
        if not args.failure_only:
            result['live'],protected=check_live(args.session,output)
        if not args.live_only:result['failure']=check_failure(output,protected)
        result['passed']=True
        print('中科生命周期实际集成检查通过；真实鼠标交互不属于此结果')
        code=0
    except (AssertionError,OSError,ValueError,KeyError,subprocess.TimeoutExpired) as exc:
        result.update(passed=False,error=str(exc));code=1
        print(f'中科生命周期检查失败：{exc}',file=sys.stderr)
    (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(f'检查结果：{output / "result.json"}')
    return code


if __name__=='__main__':
    raise SystemExit(main())
