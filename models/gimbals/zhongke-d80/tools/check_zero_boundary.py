#!/usr/bin/env python3
"""在已有独占会话验证yaw下硬端点/pitch上硬端点为0时可连续回零。"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.session import load_session
from tools.check_physics import Probe
from common import protocol


def run(probe):
    c=probe.c;m=c['mechanics'];rate=m['max_rate_dps']
    if m['yaw_min_deg']!=0 or m['pitch_max_deg']!=0:
        raise AssertionError('本夹具需要yaw下硬端点为0、pitch上硬端点为0的合法INI')
    probe.send(protocol.CMD_NOP)
    probe.wait(lambda v:v.get('mechanical_valid') and v.get('config_hash')==c['config_hash'],description='匹配配置的真实机械测量')
    probe.home('硬端点零位初始回零')
    cases=[]
    for fraction in (.25,.1):
        yaw=min(25,m['yaw_max_deg']*fraction)
        pitch=max(-25,m['pitch_min_deg']*fraction)
        probe.move_to(yaw,pitch,'从硬端点向行程内连续指定角')
        value=probe.home('从非零角连续回硬端点零位')
        if value['mode']!='IDLE' or any(abs(value['joints_deg'][axis]['pos'])>=.03 for axis in ('yaw','pitch')):
            raise AssertionError('HOME未结束或零位被软件停车缓冲挡住')
        cases.append({'target_deg':[yaw,pitch],'home_mode':value['mode'],'home_joints_deg':value['joints_deg']})
    # 在零硬端点继续请求向外，必须停止在硬范围内；命令不要求角度瞬移。
    end=time.monotonic()+.5
    while time.monotonic()<end:
        raw=protocol.quantize(min(2*rate,1000),'.1',-32768,32767)
        probe.send(protocol.CMD_MANUAL,-raw,raw)
        until=time.monotonic()+.08
        while time.monotonic()<until:probe.tick(keepalive=False)
    probe.stopped();probe.home('向外请求后仍可完成HOME')
    with probe.lock:samples=list(probe.samples)
    if len(samples)<20:raise AssertionError('真实机械采样不足')
    peaks={};differences={}
    for axis in ('yaw','pitch'):
        for row in samples:
            position=row[axis+'_pos'];velocity=row[axis+'_vel']
            if not math.isfinite(position) or not math.isfinite(velocity):raise AssertionError('非有限物理采样')
            if not m[axis+'_min_deg']-1e-5<=position<=m[axis+'_max_deg']+1e-5:
                raise AssertionError(f'{axis}真实位置越过硬边界：{position}')
            if abs(velocity)>rate+1e-4:raise AssertionError(f'{axis}实际速度超出当前配置')
        peaks[axis]=max(abs(x[axis+'_vel']) for x in samples)
        rates=[abs((b[axis+'_pos']-a[axis+'_pos'])/(b['sim_time']-a['sim_time'])) for a,b in zip(samples,samples[1:])
               if 1e-6<b['sim_time']-a['sim_time']<1]
        differences[axis]=max(rates,default=0)
        if differences[axis]>rate+1e-4:raise AssertionError(f'{axis}独立位置差分超速')
    with (probe.output/'samples.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(samples[0]));writer.writeheader();writer.writerows(samples)
    return {'result':'passed','config_hash':c['config_hash'],'hard_limits_deg':{
        'yaw':[m['yaw_min_deg'],m['yaw_max_deg']],'pitch':[m['pitch_min_deg'],m['pitch_max_deg']]},
        'stop_margin_deg':m['stop_margin_deg'],'max_rate_dps':rate,'cases':cases,
        'mechanical_peak_dps':peaks,'position_difference_peak_dps':differences,'sample_count':len(samples),
        'udp_sample_count':len(probe.received),'method':'中科标准UDP指定角/回零/手动，真实Gazebo关节采样与独立位置差分'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',default='zero_boundary');parser.add_argument('--output',type=Path)
    args=parser.parse_args();session=load_session(args.session)
    output=args.output or Path(session['session_path']).parent/'zero_boundary'
    probe=Probe(session,output)
    try:
        result=run(probe);print('中科硬端点零位连续HOME实际检查通过')
    except Exception as exc:
        result={'result':'failed','error':str(exc),'config_hash':probe.c['config_hash']}
        print(f'中科硬端点零位检查失败：{exc}',file=sys.stderr)
    finally:probe.close()
    (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return 0 if result['result']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
