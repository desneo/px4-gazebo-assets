#!/usr/bin/env python3
"""在独立02中科运行副本中验收二进制锁定；仅移动本工具创建的视觉夹具。"""
from __future__ import annotations

import argparse
import copy
import gzip
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET
import uuid

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT.parents[1]
D80 = ASSETS / 'models/gimbals/zhongke-d80'
sys.path.insert(0, str(D80))
from common.config import load_config
from common import protocol
from ground_station.binary_client import BinaryClient


def rotate(q, v):
    norm = math.sqrt(sum(x*x for x in q))
    w,x,y,z = (x/norm for x in q)
    a,b,c = v
    tx,ty,tz = 2*(y*c-z*b),2*(z*a-x*c),2*(x*b-y*a)
    return [a+w*tx+y*tz-z*ty,b+w*ty+z*tx-x*tz,c+w*tz+x*ty-y*tx]


def services(pipe, partition, world):
    """请求进程不订阅高频遥测，避免Python绑定的GIL竞争。"""
    os.environ['GZ_PARTITION'] = partition
    from gz.transport13 import Node
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.scene_pb2 import Scene
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.pose_pb2 import Pose
    node = Node()
    while True:
        request = pipe.recv()
        op = request['op']
        if op == 'quit': break
        try:
            if op == 'scene':
                ok, response = node.request(f'/world/{world}/scene/info', Empty(), Empty, Scene, 4000)
                if not ok: raise RuntimeError('读取场景实体超时')
                result = {m.name:[link.name for link in m.link] for m in response.model}
            else:
                if op == 'create':
                    payload = EntityFactory(sdf=request['sdf'],allow_renaming=False)
                elif op == 'remove':
                    payload = Entity(name=request['name'],type=Entity.MODEL)
                elif op == 'set_pose':
                    payload = Pose(name=request['name'])
                    payload.position.x,payload.position.y,payload.position.z = request['position']
                    payload.orientation.w = 1
                else: raise ValueError('未知请求：'+op)
                ok, response = node.request(f'/world/{world}/{op}', payload, type(payload), Boolean, 4000)
                if not ok or not response.data: raise RuntimeError('Gazebo请求失败：'+op)
                result = True
            pipe.send({'ok':True,'result':result})
        except Exception as exc:
            pipe.send({'ok':False,'error':str(exc)})


class Probe:
    def __init__(self, args):
        self.a = args
        self.closed = False
        self.node = self.client = self.pipe = self.worker = None
        self.rows,self.cases = [],[]
        self.created = False
        self.capture_image = None
        self.marker = 'ZHONGKE_LOCK_FIXTURE_' + uuid.uuid4().hex
        self.cfg = load_config(args.config)
        self.model = self.cfg['model']['name']
        self.name = 'xiaobianfu'
        if self.name not in self.cfg['tracking']['target_names']:
            raise ValueError('本验收需要配置注册目标xiaobianfu')
        # 明确限制到新增中科组合机与其独立Gazebo分区。
        if not self.model.startswith('rocket_zhongke_d80_') or 'zhongke' not in args.partition:
            raise ValueError('只允许新增中科组合机和显式中科分区，禁止操作原场景')
        os.environ['GZ_PARTITION'] = args.partition
        os.environ['GZ_IP'] = '127.0.0.1'
        from gz.transport13 import Node
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.msgs10.image_pb2 import Image
        self.lock = threading.RLock()
        self.latest = {}
        self.rows,self.cases = [],[]
        self.phase = '连接'
        self.created = False
        self.node = Node()
        for key,suffix in [('plugin','telemetry'),('diagnostic','diagnostics')]:
            self.node.subscribe(StringMsg,args.prefix+'/'+suffix,
                lambda msg,key=key:self.on_json(key,msg))
        self.client = BinaryClient('127.0.0.1',self.cfg['network']['udp_port'],on_telemetry=self.on_binary)
        context = mp.get_context('spawn')
        self.pipe,child = context.Pipe()
        self.worker = context.Process(target=services,args=(child,args.partition,args.world),daemon=True)
        self.worker.start();child.close()
        self.node.subscribe(Image,args.prefix+'/image',self.on_image)
        # 先证明本分区有同名同配置控制器，再向专用端口发心跳。
        deadline = time.monotonic()+30
        while time.monotonic()<deadline:
            snap = self.snapshot()
            if all(k in snap and time.monotonic()-snap[k][0]<1 for k in ('plugin','diagnostic','image')):
                break
            time.sleep(.05)
        else: raise AssertionError('未收到同源插件、诊断和实际图像，未发送UDP控制')
        self.client.start()

    def on_image(self,message):
        self.record('image',{'width':message.width,'height':message.height,
            'sim_stamp':message.header.stamp.sec+message.header.stamp.nsec*1e-9})
        destination = self.capture_image
        if destination is not None:
            self.capture_image = None
            try:
                import numpy as np
                import cv2
                raw = np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)
                pixels = raw[:,:message.width*3].reshape(message.height,message.width,3)
                destination.parent.mkdir(parents=True,exist_ok=True)
                if not cv2.imwrite(str(destination),cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR)):
                    raise RuntimeError('实际图像文件保存失败')
            except Exception as exc:
                self.record('image_error',{'error':str(exc)})

    def on_json(self,key,message):
        try: value = json.loads(message.data)
        except (ValueError,TypeError): return
        if value.get('model_name') != self.model or value.get('config_hash') != self.cfg['config_hash']: return
        self.record(key,value)

    def on_binary(self,sample):
        self.record('binary',{'fields':dict(sample.fields),'raw_hex':sample.raw.hex(),'rx_seq':sample.rx_seq})

    def record(self,key,value):
        with self.lock:
            now = time.monotonic()
            self.latest[key] = (now,value)
            self.rows.append({'kind':key,'wall_time':now,'phase':self.phase,'value':value})

    def request(self,op,**values):
        self.pipe.send({'op':op,**values})
        if not self.pipe.poll(7): raise AssertionError('请求进程超时：'+op)
        response = self.pipe.recv()
        if not response['ok']: raise AssertionError(response['error'])
        self.record('service',{'op':op,**values})
        return response['result']

    def snapshot(self):
        with self.lock: return copy.deepcopy(self.latest)

    def wait(self,predicate,label,timeout=12,after=0):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            snap = self.snapshot()
            if all(key in snap and snap[key][0]>after and time.monotonic()-snap[key][0]<1.0
                   for key in ('plugin','diagnostic','binary','image')):
                value = {k:v[1] for k,v in snap.items()}
                if predicate(value): return value
            time.sleep(.02)
        snap = self.snapshot()
        raise AssertionError(label+'超时：'+str({k:v[1] for k,v in snap.items()}))

    def case(self,label,predicate,timeout=12,after=0):
        value = self.wait(predicate,label,timeout,after)
        self.cases.append({'name':label,'passed':True,'snapshot':value})
        print(label+'：通过',flush=True)
        return value

    def send(self,command,**values):
        self.record('command',{'command':command,**values})
        if not self.client.send(command,**values): raise RuntimeError('UDP发送失败')
        return time.monotonic()

    @staticmethod
    def candidate(value):
        return next((c for c in value['plugin'].get('truth_candidates',[]) if c['name']=='xiaobianfu'),None)

    def place(self,position):
        self.request('set_pose',name=self.name,position=position)
        return self.wait(lambda s:(c:=self.candidate(s)) is not None and
            max(abs(a-b) for a,b in zip(c['entity_position'],position))<1e-4,'视觉夹具姿态同步')

    def locked(self,s,selection):
        return (s['diagnostic'].get('selection_mode')==selection and
            s['diagnostic'].get('target_name')==self.name and s['diagnostic'].get('target_valid') is True and
            s['plugin'].get('target',{}).get('entity_name')==self.name and
            s['plugin'].get('target',{}).get('valid') is True and
            bool(s['binary']['fields']['status1']&4))

    def run(self):
        self.case('同源实体、配置与新鲜二进制遥测',lambda s:s['plugin'].get('mechanical_valid') is True and
            s['plugin'].get('camera_ready') is True and s['plugin'].get('mount',{}).get('static') is False and
            s['image']['width']==self.cfg['optics']['width_px'] and
            s['image']['height']==self.cfg['optics']['height_px'] and all(
                abs(s['binary']['fields']['frame_'+axis]-s['plugin']['joints_deg'][axis]['pos'])<.5
                for axis in ('yaw','pitch')),timeout=30)
        if self.name in self.request('scene'): raise AssertionError('场景已有xiaobianfu，拒绝移动或删除现有目标')
        # 机械零位向上，前视由真实内轴转到-70°；不修改安装外参。
        self.phase = '前向观察'
        stamp = self.send(protocol.CMD_FRAME_ANGLE,y=-7000)
        self.case('俯仰实际运动至前向观察角',lambda s:abs(s['binary']['fields']['frame_pitch']+70)<.2,timeout=22,after=stamp)
        source = ET.parse(ASSETS/'models/vehicles/xiaobianfu/model.sdf').getroot()
        model = source.find('model');model.set('name',self.name)
        for parent in model.iter():
            for child in list(parent):
                if child.tag in ('sensor','collision','plugin'): parent.remove(child)
        static = model.find('static')
        if static is None: static = ET.SubElement(model,'static')
        static.text = 'true'
        pose = model.find('pose')
        if pose is None: pose = ET.SubElement(model,'pose')
        pose.text = '0 0 500 0 0 0'
        ET.SubElement(model,'link',name=self.marker)
        self.created = True  # 即使响应丢失，也按唯一link标记确认所有权后清理。
        self.request('create',sdf=ET.tostring(source,encoding='unicode'))
        value = self.wait(lambda s:self.candidate(s) is not None,'新创建视觉夹具同步')
        center = self.candidate(value)['camera_optical_center']
        q = value['plugin']['camera']['quat_world']
        axis = rotate(q,[1,0,0]);side = rotate(q,[0,1,0])
        distance = min(150.,self.cfg['tracking']['discovery_range_m']*.5)
        origin = [c+distance*d for c,d in zip(center,axis)]
        value = self.place(origin)
        c = self.candidate(value)
        if not c['valid'] or c['range_m']>self.cfg['tracking']['discovery_range_m']: raise AssertionError('夹具未在配置发现距离内的当前视场')
        self.phase = '自动锁定'
        stamp = self.send(protocol.CMD_AUTO_LOCK)
        self.case('标准0F自动锁定同一真实目标',lambda s:self.locked(s,'AUTO'),after=stamp)
        self.capture_image = self.a.output/'auto_lock.png'
        start_pitch = value['binary']['fields']['frame_pitch']
        self.phase = '目标移动'
        for i in range(1,9):
            position = [o+side[k]*i*.3+(0. if k<2 else i*.2) for k,o in enumerate(origin)]
            self.place(position)
            time.sleep(.08)
        after_move = time.monotonic()
        self.case('移动后保持同一目标并收敛角偏差',lambda s:self.locked(s,'AUTO') and
            max(abs(s['binary']['fields'][k]) for k in ('offset_x','offset_y'))<.35,timeout=12,after=after_move)
        changed = self.snapshot()['binary'][1]['fields']['frame_pitch']
        if abs(changed-start_pitch)<.1: raise AssertionError('移动目标未引起真实关节角变化')
        self.phase = '指点锁定'
        self.send(protocol.CMD_STOP_TRACK)
        value = self.wait(lambda s:not s['binary']['fields']['status1']&4,'停止自动锁定')
        c = self.candidate(value)
        stamp = self.send(protocol.CMD_POINT_TRACK,x=round(c['center_px'][0]-self.cfg['optics']['width_px']/2),
            y=round(c['center_px'][1]-self.cfg['optics']['height_px']/2))
        self.case('标准0D指点锁定同一真实目标',lambda s:self.locked(s,'POINT'),after=stamp)
        self.phase = '人工接管'
        stamp = self.send(protocol.CMD_MANUAL,x=10)
        self.case('手动接管撤销目标绑定',lambda s:s['diagnostic'].get('selection_mode')=='OFF' and
            s['diagnostic'].get('target_name') is None and not s['binary']['fields']['status1']&4,after=stamp)
        self.send(protocol.CMD_MANUAL)
        self.phase = '回零'
        stamp = self.send(protocol.CMD_HOME)
        self.case('真实两轴回零完成',lambda s:max(abs(s['binary']['fields'][k]) for k in ('frame_yaw','frame_pitch'))<.1 and
            s['plugin'].get('mode')=='IDLE',timeout=22,after=stamp)
        with self.lock:
            rates = [abs(row['value']['fields'][key]) for row in self.rows if row['kind']=='binary'
                     for key in ('yaw_rate','pitch_rate')]
        peak = max(rates)
        if peak>self.cfg['mechanics']['max_rate_dps']+.05: raise AssertionError('实际速度超过配置上限')
        return {'passed':True,'cases':self.cases,'maximum_measured_axis_rate_dps':peak,
            'fixture_range_m':distance,'model':self.model,'config_hash':self.cfg['config_hash'],
            'partition':self.a.partition,'world':self.a.world,'prefix':self.a.prefix,
            'scope':'02地理世界中PX4待命载机、真实云台关节与相机；仅视觉夹具运动，未验证载机飞行',
            'protocol':'16字节中科控制、32字节中科遥测；目标身份由同分区只读诊断核验'}

    def close(self):
        if self.closed: return []
        self.closed = True
        errors = []
        try:
            if self.created:
                scene = self.request('scene')
                if self.name in scene:
                    if self.marker not in scene[self.name]:
                        raise RuntimeError('同名实体缺少本次唯一标记，拒绝删除')
                    self.request('remove',name=self.name)
                    deadline = time.monotonic()+3
                    while self.name in self.request('scene'):
                        if time.monotonic()>deadline: raise RuntimeError('视觉夹具删除后仍存在')
                        time.sleep(.05)
        except Exception as exc: errors.append('夹具清理：'+str(exc))
        try:
            if self.client is not None: self.client.stop()
        except Exception as exc: errors.append('UDP清理：'+str(exc))
        try:
            if self.node is not None:
                for suffix in ('telemetry','diagnostics','image'):
                    self.node.unsubscribe(self.a.prefix+'/'+suffix)
        except Exception as exc: errors.append('订阅清理：'+str(exc))
        if self.worker is not None and self.worker.pid is not None:
            try:
                self.pipe.send({'op':'quit'})
                self.worker.join(2)
                if self.worker.is_alive(): self.worker.terminate();self.worker.join(2)
            except (OSError,EOFError) as exc: errors.append('请求进程清理：'+str(exc))
        if self.pipe is not None: self.pipe.close()
        self.a.output.mkdir(parents=True,exist_ok=True)
        with gzip.open(self.a.output/'samples.jsonl.gz','wt',encoding='utf-8') as stream:
            for row in self.rows: stream.write(json.dumps(row,ensure_ascii=False)+'\n')
        return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--world',default='geo_32_217857_119_105878')
    parser.add_argument('--partition',required=True)
    parser.add_argument('--prefix',default='/uav_1/pod')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    probe = Probe.__new__(Probe)
    try:
        probe.__init__(args)
        result = probe.run()
    except Exception as exc:
        result = {'passed':False,'error':str(exc),'cases':probe.cases}
        print('场景锁定验收失败：'+str(exc),file=sys.stderr)
    finally:
        errors = probe.close()
    if errors:
        result['passed'] = False
        result['cleanup_errors'] = errors
    (args.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return 0 if result['passed'] else 1

if __name__ == '__main__': raise SystemExit(main())
