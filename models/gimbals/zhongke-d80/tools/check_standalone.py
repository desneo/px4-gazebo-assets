#!/usr/bin/env python3
"""使用独立UDP测试端和已有RTSP验证view config.ini启动；不操纵运行中的云台。"""
from __future__ import annotations
import argparse
import configparser
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from common.protocol import encode_telemetry,decode_command
from scripts.session import load_session,process_matches


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rtsp-url',default='rtsp://127.0.0.1:8556/zhongke_d80_1')
    parser.add_argument('--gui',action='store_true',help='在真实显示环境检查完整窗口')
    args=parser.parse_args()
    directory=ROOT/'runs'/('standalone_gui_check' if args.gui else 'standalone_check')
    directory.mkdir(parents=True,exist_ok=True)
    config=directory/'config.ini'
    peer=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    peer.bind(('127.0.0.1',0));peer.settimeout(.004)
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as reservation:
        reservation.bind(('127.0.0.1',0));local_port=reservation.getsockname()[1]
    prefs=configparser.ConfigParser(interpolation=None)
    prefs.read(ROOT/'ground_station/config.ini')
    prefs['connection']=dict(udp_host='127.0.0.1',udp_port=str(peer.getsockname()[1]),
        local_bind='127.0.0.1',local_port=str(local_port),rtsp_url=args.rtsp_url,
        parameters_file=str(ROOT/'parameters.ini'))
    prefs['video']['window_title']='中科 D80 独立连接检查'
    with config.open('w') as stream:prefs.write(stream)
    values=dict(status1=0,status2=8,status3=0,zoom=1,offset_x=0,offset_y=0,
        frame_roll=0,frame_pitch=0,frame_yaw=0,roll_rate=0,pitch_rate=0,yaw_rate=0,
        laser_range=0,self_check=128)
    packet=encode_telemetry(values)
    done=threading.Event();received=[];sources=set();transmitted=0
    def serve():
        nonlocal transmitted
        destination=None;next_send=0.
        while not done.is_set():
            try:
                raw,address=peer.recvfrom(128)
                command=decode_command(raw)
                received.append(command);sources.add(address);destination=address
            except socket.timeout:pass
            if destination and time.monotonic()>=next_send:
                peer.sendto(packet,destination);transmitted+=1
                next_send=time.monotonic()+1/60
    worker=threading.Thread(target=serve,daemon=True);worker.start()
    command=['bash',str(ROOT/'ground_station/run.sh'),'view',str(config),
        '--duration','4','--snapshot',str(directory/'station.png')]
    if not args.gui:command.append('--selftest')
    result={'passed':False,'command':command,'test_source':'独立32字节UDP测试报文，非物理测量验收'}
    try:
        checked=subprocess.run(command,cwd=ROOT/'ground_station',capture_output=True,text=True,timeout=25)
        output=checked.stdout+checked.stderr
        (directory/'launch.log').write_text(output)
        if checked.returncode:raise AssertionError(f'独立启动退出码{checked.returncode}，请查看launch.log')
        if not (directory/'station.png').exists():raise AssertionError('未取得实际解码/窗口截图')
        if len(received)<5 or transmitted<30:raise AssertionError('独立客户端未持续收发标准UDP')
        if sources!={('127.0.0.1',local_port)}:raise AssertionError(f'控制与遥测未使用配置的同一端口：{sources}')
        from ground_station.configuration import client_manifest_path
        registry=client_manifest_path(config)
        manifest=load_session(registry)
        if process_matches(manifest['processes']['ground_station']):raise AssertionError('客户端退出后仍存活')
        stopped=subprocess.run(['bash',str(ROOT/'ground_station/stop.sh'),'--station-config',str(config)],
            capture_output=True,text=True,timeout=5)
        if stopped.returncode:raise AssertionError('独立客户端重复停止失败')
        result.update(passed=True,device_udp=list(peer.getsockname()),local_udp=['127.0.0.1',local_port],
            valid_command_count=len(received),telemetry_sent=transmitted,registry=str(registry),
            exit_code=checked.returncode,requires_world_manifest=False,rtsp_url=args.rtsp_url)
    except Exception as exc:
        result['error']=str(exc)
    finally:
        done.set();worker.join(timeout=2);peer.close()
    (directory/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
