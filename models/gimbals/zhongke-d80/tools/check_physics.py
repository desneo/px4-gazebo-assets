#!/usr/bin/env python3
"""对已有中科会话执行真实UDP操纵与Gazebo物理/CameraInfo验收；不启动世界。"""
from __future__ import annotations
import argparse
import csv
import json
import math
import os
from pathlib import Path
import socket
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.session import load_session
from common.config import load_config
from common import protocol


class Probe:
    def __init__(self,session,output):
        self.session=session;self.output=output;output.mkdir(parents=True,exist_ok=True)
        self.c=load_config(session['config_path'])
        os.environ['GZ_PARTITION']=session['gz_partition']
        from gz.transport13 import Node
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.msgs10.camera_info_pb2 import CameraInfo
        from gz.msgs10.image_pb2 import Image
        self.node=Node();self.lock=threading.Lock();self.latest={};self.samples=[]
        self.camera_info={};self.image=None;self.received=[];self.results=[]
        self.last_nop=0.;self.bad_udp=0
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.bind(('127.0.0.1',0));self.sock.setblocking(False)
        self.destination=(session['udp_host'],int(session['udp_port']))
        prefix=session.get('topic_prefix','/zhongke_d80')
        self.node.subscribe(StringMsg,prefix+'/telemetry',self.on_telemetry)
        self.node.subscribe(CameraInfo,prefix+'/camera_info',self.on_camera_info)
        self.node.subscribe(Image,session.get('image_topic',prefix+'/camera'),self.on_image)

    def on_telemetry(self,msg):
        try:
            value=json.loads(msg.data)
            joints=value['joints_deg'];stamp=float(value['ts_mono'])
            sample={'sim_time':stamp,'wall_time':time.monotonic(),'mode':value['mode']}
            for axis in ('yaw','pitch'):
                sample[axis+'_pos']=float(joints[axis]['pos'])
                sample[axis+'_vel']=float(joints[axis]['vel'])
            with self.lock:
                self.latest=value
                if not self.samples or self.samples[-1]['sim_time']!=stamp:
                    self.samples.append(sample)
        except (KeyError,ValueError,TypeError):
            pass

    def on_camera_info(self,msg):
        with self.lock:
            self.camera_info={'width':msg.width,'height':msg.height,'k':list(msg.intrinsics.k),
                              'p':list(msg.projection.p),'wall_time':time.monotonic()}

    def on_image(self,msg):
        with self.lock:
            self.image=(time.monotonic(),msg.width,msg.height,msg.step,msg.pixel_format_type,bytes(msg.data))

    def snapshot(self):
        with self.lock:return dict(self.latest)

    def send(self,cmd,x=0,y=0):
        self.sock.sendto(protocol.encode_command(cmd,x,y),self.destination)
        self.last_nop=time.monotonic()

    def tick(self,keepalive=True):
        if keepalive and time.monotonic()-self.last_nop>.15:self.send(protocol.CMD_NOP)
        while True:
            try:data,source=self.sock.recvfrom(65535)
            except BlockingIOError:break
            if source!=self.destination:continue
            try:value=protocol.decode_telemetry(data)
            except ValueError:self.bad_udp+=1;continue
            self.received.append({'wall_time':time.monotonic(),**value})
        time.sleep(.01)

    def wait(self,predicate,timeout=8,description='等待状态'):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.tick()
            value=self.snapshot()
            if value and predicate(value):return value
        raise AssertionError(f'{description}超时：{self.snapshot()}')

    def stopped(self):
        before=self.snapshot().get('ts_mono',-1)
        self.send(protocol.CMD_STOP_TRACK);self.send(protocol.CMD_MANUAL,0,0)
        return self.wait(lambda v:v['ts_mono']>before and v['mode']=='IDLE' and
                         all(abs(v['joints_deg'][a]['vel'])<.05 for a in ('yaw','pitch')),description='停止')

    def move_to(self,yaw,pitch,label,endpoint=False):
        now=self.snapshot();before=now.get('ts_mono',-1)
        distance=max(abs(now.get('joints_deg',{}).get(a,{}).get('pos',0)-p) for a,p in [('yaw',yaw),('pitch',pitch)])
        m=self.c['mechanics'];rate=m['max_rate_dps']
        timeout=distance/rate+2*rate/m['max_acc_dps2']+8
        self.send(protocol.CMD_FRAME_ANGLE,protocol.quantize(yaw,'.01',-32768,32767),protocol.quantize(pitch,'.01',-32768,32767))
        tolerance=m['stop_margin_deg']+.04 if endpoint else .08
        value=self.wait(lambda v:v['ts_mono']>before and
            abs(v['joints_deg']['yaw']['pos']-yaw)<=tolerance and
            abs(v['joints_deg']['pitch']['pos']-pitch)<=tolerance and
            all(abs(v['joints_deg'][a]['vel'])<.05 for a in ('yaw','pitch')),
            timeout,description=label)
        self.results.append({'case':label,'joints_deg':value['joints_deg'],'singular':value.get('singular')})
        return value

    def home(self,label='回零'):
        before=self.snapshot().get('ts_mono',-1);m=self.c['mechanics']
        distance=max(abs(self.snapshot().get('joints_deg',{}).get(a,{}).get('pos',0)) for a in ('yaw','pitch'))
        self.send(protocol.CMD_HOME)
        value=self.wait(lambda v:v['ts_mono']>before and v['mode']=='IDLE' and
            all(abs(v['joints_deg'][a]['pos'])<.08 and abs(v['joints_deg'][a]['vel'])<.05 for a in ('yaw','pitch')),
            distance/m['max_rate_dps']+10,description=label)
        self.results.append({'case':label,'joints_deg':value['joints_deg']})
        return value

    def capture(self,label):
        started=time.monotonic()
        self.wait(lambda _:self.image is not None and self.image[0]>started,timeout=5,description='真实相机新图像')
        with self.lock:_,width,height,step,pixel_format,data=self.image
        from PIL import Image
        if len(data)<step*height:raise AssertionError('真实相机图像长度不足')
        # 本组件SDF配置为RGB8；按真实row stride保存，不能用离线重绘替代证据。
        if step<width*3:raise AssertionError('相机不是可保存的RGB8图像')
        path=self.output/(label+'.png')
        Image.frombytes('RGB',(width,height),data,'raw','RGB',step,1).save(path)
        self.results.append({'case':label,'image':str(path),'width':width,'height':height,'pixel_format':pixel_format})

    def run(self):
        self.send(protocol.CMD_NOP)
        self.wait(lambda v:v.get('config_hash')==self.c['config_hash'] and v.get('mechanical_valid'),description='匹配配置的新机械遥测')
        self.wait(lambda _:bool(self.received),description='真实32字节UDP遥测')
        self.wait(lambda _:bool(self.camera_info),description='真实CameraInfo')
        with self.lock:info=dict(self.camera_info)
        k=info['k'];d=self.c['derived'];o=self.c['optics']
        if len(k)!=9:raise AssertionError(f'CameraInfo K不是3×3：{k}')
        for index,key in [(0,'fx'),(4,'fy'),(2,'cx'),(5,'cy')]:
            if not math.isclose(k[index],d[key],rel_tol=1e-7,abs_tol=1e-5):
                raise AssertionError(f'真实CameraInfo {key}与统一配置不符：{k[index]} / {d[key]}')
        if (info['width'],info['height'])!=(o['width_px'],o['height_px']):raise AssertionError('实际CameraInfo分辨率不符')
        self.home();self.capture('zero_actual_render')
        rate=self.c['mechanics']['max_rate_dps']
        for sign,label in [(1,'人工右上'),(-1,'人工左下')]:
            end=time.monotonic()+max(1.,rate/self.c['mechanics']['max_acc_dps2']+.3)
            while time.monotonic()<end:
                request=min(2*rate,1000)*sign
                raw=protocol.quantize(request,'.1',-32768,32767)
                self.send(protocol.CMD_MANUAL,raw,raw)
                until=time.monotonic()+.08
                while time.monotonic()<until:self.tick(keepalive=False)
            self.stopped();self.results.append({'case':label,'joints_deg':self.snapshot()['joints_deg']})
        # 手动停止计时已清理；回零持续时间超过0.5s，验证旧MANUAL计时不会抢停新HOME。
        self.move_to(15,-20,'指定框架角')
        self.home('指定角后连续回零')
        m=self.c['mechanics']
        for axis,key,label in [('yaw','yaw_max_deg','方位正端点'),('yaw','yaw_min_deg','方位负端点'),
                               ('pitch','pitch_min_deg','俯仰负端点'),('pitch','pitch_max_deg','俯仰正端点')]:
            yaw=m[key] if axis=='yaw' else 0;pitch=m[key] if axis=='pitch' else 0
            value=self.move_to(yaw,pitch,label,endpoint=True)
            if axis=='pitch' and abs(pitch+90)<1e-6 and not value.get('singular'):
                raise AssertionError('俯仰−90°附近未报告视线退化')
            self.capture(label+'_actual_render')
        self.home('端点后回零');self.stopped()
        with self.lock:samples=list(self.samples)
        if len(samples)<20:raise AssertionError('物理样本不足')
        peak={};difference_peak={};angle_epsilon=1e-5;rate_epsilon=1e-4
        for axis in ('yaw','pitch'):
            peak[axis]=max(abs(s[axis+'_vel']) for s in samples)
            for sample in samples:
                p=sample[axis+'_pos'];v=sample[axis+'_vel']
                if not math.isfinite(p) or not math.isfinite(v):raise AssertionError('非有限物理测量')
                if not m[axis+'_min_deg']-angle_epsilon<=p<=m[axis+'_max_deg']+angle_epsilon:
                    raise AssertionError(f'{axis}实际位置越过硬边界：{p}')
                if abs(v)>rate+rate_epsilon:raise AssertionError(f'{axis}原始实测速率超限：{v}>{rate}')
            rates=[abs((b[axis+'_pos']-a[axis+'_pos'])/(b['sim_time']-a['sim_time'])) for a,b in zip(samples,samples[1:])
                   if 1e-6<b['sim_time']-a['sim_time']<1]
            difference_peak[axis]=max(rates,default=0)
            if difference_peak[axis]>rate+rate_epsilon:raise AssertionError(f'{axis}独立位置差分超限：{difference_peak[axis]}')
            if peak[axis]<.9*rate:raise AssertionError(f'{axis}未实际到达配置限速，不能证明改参生效')
        with (self.output/'samples.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(samples[0]));writer.writeheader();writer.writerows(samples)
        return {'result':'passed','session':self.session['session_path'],'config_hash':self.c['config_hash'],
                'max_rate_dps':rate,'mechanical_peak_dps':peak,'position_difference_peak_dps':difference_peak,
                'sample_count':len(samples),'udp_sample_count':len(self.received),'bad_udp_count':self.bad_udp,
                'measurement_tolerance':{'angle_deg':angle_epsilon,'rate_dps':rate_epsilon},'camera_info':info,
                'cases':self.results,'geometry_review':'保存实际渲染用于端点遮挡/穿插人工复核；本工具不自动证明几何无相交'}

    def close(self):
        try:self.send(protocol.CMD_STOP_TRACK);self.send(protocol.CMD_MANUAL,0,0)
        except OSError:pass
        self.sock.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',default='validation');parser.add_argument('--output',type=Path)
    args=parser.parse_args();session=load_session(args.session)
    output=args.output or Path(session['session_path']).parent/'physics'
    probe=Probe(session,output)
    try:
        result=probe.run();print('中科真实物理、UDP与CameraInfo检查通过')
    except Exception as exc:
        result={'result':'failed','error':str(exc),'cases':probe.results,'sample_count':len(probe.samples)}
        print(f'中科真实物理检查失败：{exc}',file=sys.stderr)
    finally:
        probe.close()
    (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return 0 if result['result']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
