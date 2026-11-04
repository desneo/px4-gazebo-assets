#!/usr/bin/env python3
"""通过实际 UDP 验证中科客户端、转发生命周期与 JSON 接收结果。"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import socket
import struct
import sys
import threading
import time
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from common.config import load_config
from ground_station.binary_client import BinaryClient
from ground_station.telemetry_forward import TelemetryForwarder, FIELDS, ForwardInputs
from scripts.session import load_session


def wait_until(predicate,timeout=3):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('等待中科遥测条件超时')


class Collector:
    def __init__(self):
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1',0))
        self.socket.settimeout(.05)
        self.address=self.socket.getsockname()
        self.rows=[]
        self.error=''
        self.lock=threading.Lock()
        self.stopped=threading.Event()
        self.thread=threading.Thread(target=self._run,daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stopped.is_set():
            try:
                raw,_=self.socket.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                value=json.loads(raw.decode('utf-8'))
                validate_json(value)
            except (ValueError,UnicodeError,AssertionError) as exc:
                self.error=str(exc)
                continue
            with self.lock:
                self.rows.append((time.monotonic(),time.time_ns()//1_000_000,value))

    def snapshot(self):
        with self.lock:
            return list(self.rows)

    def close(self):
        self.stopped.set()
        self.socket.close()
        self.thread.join(timeout=1)


def validate_json(value):
    assert isinstance(value,dict) and set(value)==set(FIELDS)|{'timestamp'},'JSON 必须恰好包含15字段'
    assert type(value['timestamp']) is int and value['timestamp']>=0,'timestamp 必须为Unix整数毫秒'
    for key in ('status1','status2','status3','self_check'):
        assert type(value[key]) is int and 0<=value[key]<=255,f'{key}未保留原始字节'
    for key in set(FIELDS)-{'status1','status2','status3','self_check'}:
        assert type(value[key]) in (int,float) and math.isfinite(value[key]),f'{key}不是有限数值'
    delta=time.time_ns()//1_000_000-value['timestamp']
    assert -100<=delta<=2000,'timestamp 与实际接收的Unix毫秒不一致'


def pause_for_edit(forward):
    # 调用GUI正在使用的暂停/编辑方法，确认Enter不会隐式恢复发送。
    from ground_station.ground_station import GroundStation
    holder=SimpleNamespace(forwarder=forward,forward_note='',forward_inputs=ForwardInputs('127.0.0.1',9000))
    holder._pause_forward_for_edit=lambda: GroundStation._pause_forward_for_edit(holder)
    holder.forward_inputs.activate('ip')
    GroundStation._handle_forward_key(holder,13)
    assert holder.forward_inputs.focus is None,'Enter没有结束地址编辑'
    assert not forward.snapshot()['active'] and forward.snapshot()['queued']==0,'编辑地址未停发并清队列'
    assert forward.snapshot()['send_hz']==0,'停发后发送频率没有立即归零'


def assert_silent(collector,duration=.15):
    time.sleep(.05)  # 接收已在暂停前交给内核的包，再验证后续没有新增。
    before=len(collector.snapshot())
    time.sleep(duration)
    assert len(collector.snapshot())==before,'停发、断流或重启时重放旧测量'
    assert not collector.error,collector.error


def golden_frame():
    # 独立按厂商偏移构造，避免仅编码器/解码器往返自证。
    frame=bytearray(32)
    frame[:6]=bytes((0xEE,0x16,4,8,10,0xA0))
    for offset,raw in ((6,-20),(8,30),(12,-1234),(14,4321),(22,2150),(24,-4567)):
        struct.pack_into('<h',frame,offset,raw)
    frame[28]=128
    frame[31]=sum(frame[:31])&255
    return bytes(frame)


def offline_check():
    collector=Collector()
    forward=TelemetryForwarder(max_age_s=.2)
    source=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    source.bind(('127.0.0.1',0))
    source.settimeout(2)
    samples=[]
    def offer(sample):
        samples.append(sample)
        forward.offer(sample)
    client=BinaryClient(*source.getsockname(),max_age_s=.2,on_telemetry=offer)
    try:
        client.start()
        command,peer=source.recvfrom(128)
        assert len(command)==16 and command[:3]==b'\xeb\x90\0','客户端未使用16字节空指令'
        packet=golden_frame()
        source.sendto(packet,peer)
        wait_until(lambda:len(samples)>=1)
        assert not forward.snapshot()['active'],'转发器启动时默认开启'
        assert_silent(collector)
        forward.start(*collector.address)
        assert_silent(collector,.05)
        source.sendto(packet,peer)
        source.sendto(packet,peer)
        wait_until(lambda:len(collector.snapshot())==2)
        rows=collector.snapshot()
        for _,_,row in rows:
            assert row['yaw_rate']==-45.67 and row['pitch_rate']==21.5,'实测角速度被20°/s限幅'
            assert row['status3']==0xA0 and row['zoom']==1,'状态字节或zoom解码错误'
        assert len(samples)==3 and samples[1].rx_seq!=samples[2].rx_seq,'相同内容的新采样被去重'
        pause_for_edit(forward)
        source.sendto(packet,peer)
        wait_until(lambda:len(samples)==4)
        assert_silent(collector)
        forward.start(*collector.address)
        assert_silent(collector,.05)
        source.sendto(packet,peer)
        wait_until(lambda:len(collector.snapshot())==3)
        wait_until(lambda:not client.stats()['connected'],timeout=1)
        assert_silent(collector)
        stale=replace(samples[-1],rx_seq=samples[-1].rx_seq+1,
                      received_monotonic=time.monotonic()-1)
        forward.offer(stale)
        assert_silent(collector)
        corrupt=bytearray(packet)
        corrupt[-1]^=1
        source.sendto(corrupt,peer)
        wait_until(lambda:client.stats()['rejected']>=1)
        assert len(samples)==5,'损坏帧刷新了有效接收序号'
        return dict(passed=True,json_fields=15,default_off=True,edit_pauses=True,
                    identical_datagrams_are_samples=True,stale_not_replayed=True,
                    measured_yaw_rate=-45.67,measured_pitch_rate=21.5,
                    valid_binary_received=len(samples),json_received=len(collector.snapshot()))
    finally:
        client.stop()
        forward.close()
        collector.close()
        source.close()


def live_check(session,duration):
    cfg=load_config(session['config_path'])
    assert cfg['config_hash']==session['config_hash'],'运行会话和配置hash不一致'
    collector=Collector()
    forward=TelemetryForwarder(max_age_s=cfg['network']['telemetry_max_age_s'])
    source_samples=[]
    lock=threading.Lock()
    def offer(sample):
        with lock:
            source_samples.append(sample)
        forward.offer(sample)
    client=BinaryClient(session['udp_host'],session['udp_port'],
                        max_age_s=cfg['network']['telemetry_max_age_s'],on_telemetry=offer)
    try:
        client.start()
        wait_until(lambda:client.stats()['connected'],timeout=5)
        assert not forward.snapshot()['active'],'转发器没有默认关闭'
        assert_silent(collector)
        first_seq=client.stats()['received']
        start=time.monotonic()
        forward.start(*collector.address)
        while time.monotonic()-start<duration:
            time.sleep(.05)
        elapsed=time.monotonic()-start
        last_seq=client.stats()['received']
        pause_for_edit(forward)
        assert_silent(collector,.3)
        rows=collector.snapshot()
        assert rows,'实际会话没有收到JSON'
        rate=len(rows)/elapsed
        receive_rate=(last_seq-first_seq)/elapsed
        target=cfg['network']['telemetry_hz']
        assert abs(rate-target)<=target*.05,f'实际JSON接收频率{rate:.2f}Hz超出目标{target:g}Hz±5%'
        assert abs(receive_rate-target)<=target*.05,f'实际二进制接收频率{receive_rate:.2f}Hz超出目标±5%'
        with lock:
            fields=[dict(item.fields) for item in source_samples]
        for _,_,row in rows:
            values={key:row[key] for key in FIELDS}
            assert values in fields,'JSON不是实际接收的二进制测量'
        before=len(rows)
        forward.start(*collector.address)
        wait_until(lambda:len(collector.snapshot())>before)
        forward.stop()
        assert_silent(collector,.2)
        return dict(passed=True,session=session['session_path'],duration_s=elapsed,
                    target_hz=target,binary_receive_hz=receive_rate,json_receive_hz=rate,
                    valid_binary_received=last_seq-first_seq,json_received=len(rows),
                    json_fields=15,default_off=True,edit_pauses=True,
                    invalid_frames=client.stats()['rejected'])
    finally:
        client.stop()
        forward.close()
        collector.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',default='demo')
    parser.add_argument('--duration',type=float,default=10,help='实时采样秒数，至少2，默认10')
    parser.add_argument('--offline',action='store_true',help='只执行本地黄金帧真实UDP链，不连接仿真')
    parser.add_argument('--output',type=Path,help='验收JSON保存路径')
    args=parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration<2:
        parser.error('采样秒数须为至少2的有限数')
    result={'offline':None,'live':None}
    code=0
    try:
        result['offline']=offline_check()
        if not args.offline:
            result['live']=live_check(load_session(args.session),args.duration)
        result['passed']=True
        print('中科遥测链路验收通过')
    except (AssertionError,OSError,ValueError,KeyError) as exc:
        result.update(passed=False,error=str(exc))
        code=1
        print(f'中科遥测链路验收失败：{exc}',file=sys.stderr)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return code


if __name__=='__main__':
    raise SystemExit(main())
