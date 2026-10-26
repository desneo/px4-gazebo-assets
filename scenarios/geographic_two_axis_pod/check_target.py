#!/usr/bin/env python3
"""两轴立装在完整地理背景中的动态靶标跟踪和实体遮挡验收。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import threading
import time

from prepare_world import target_model

ROOT = Path(__file__).resolve().parents[2]
WORLD = 'geographic_two_axis_pod'
INITIAL_POSE = tuple(map(float, target_model().findtext('pose').split()))
INITIAL = INITIAL_POSE[:3]
HIDDEN_PANEL = (100., 100., 80.)
_spec = importlib.util.spec_from_file_location('pod_scene_kinematics',
    ROOT / 'assembledmodels/rocket_two_axis_pod/tools/pod_kinematics.py')
_kinematics = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_kinematics)
POD_KINEMATICS = _kinematics.PodKinematics()
MOUNT_Z = 50.


def initial_angles():
    result = POD_KINEMATICS.inverse(INITIAL, MOUNT_Z)
    if not result['reachable'] or result['singular']:
        raise ValueError('初始靶标不在两轴机械可达域内')
    return result['yaw'], result['pitch']


def set_pose(name, xyz, rpy=(0., 0., 0.)):
    x,y,z=xyz
    roll,pitch,yaw=rpy
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    qw,qx,qy,qz=(cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
                 cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy)
    request=(f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
             f'orientation: {{w: {qw}, x: {qx}, y: {qy}, z: {qz}}}')
    result=subprocess.run(['gz','service','-s',f'/world/{WORLD}/set_pose',
        '--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean',
        '--timeout','5000','--req',request],capture_output=True,text=True,timeout=7,check=True)
    if 'data: true' not in result.stdout:
        raise RuntimeError(f'位姿服务未确认 {name}: {result.stdout} {result.stderr}')


def tracked(t):
    return t.get('tracking',{}).get('state')=='TRACK' and t.get('target',{}).get('valid',False)


def percentile(values, fraction=.95):
    values=sorted(values)
    return values[min(len(values)-1,int(len(values)*fraction))] if values else float('inf')


def truth_error_deg(t, odometry):
    """用光学四元数与帧时刻目标中心真值作独立验收，不反馈控制。"""
    import numpy as np
    stamp=t['frame']['sim_stamp']
    point=min(odometry,key=lambda v:abs(v['sim_time']-stamp))
    if abs(point['sim_time']-stamp)>.2:
        raise AssertionError('目标里程计与图像相差超过0.2仿真秒')
    w,x,y,z=t['camera']['quat_world']
    rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    bx,by,bw,bh=t['target']['box_px']
    f=960/math.tan(math.pi/6)
    ray=rotation @ np.array([1,-(bx+bw/2-960)/f,-(by+bh/2-540)/f])
    origin=POD_KINEMATICS.camera_pose(t['joints_deg']['yaw']['pos'],
        t['joints_deg']['pitch']['pos'], MOUNT_Z)[:3,3]
    truth=np.array([point['x'],point['y'],point['z']])-origin
    cosine=np.dot(ray,truth)/(np.linalg.norm(ray)*np.linalg.norm(truth))
    return math.degrees(math.acos(float(np.clip(cosine,-1,1))))


class Session:
    def __init__(self, args):
        from gz.transport13 import Node
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.twist_pb2 import Twist
        self.args=args;self.Twist=Twist;self.node=Node()
        self.pub=self.node.advertise('/model/tracking_target/cmd_vel',Twist)
        self.address=(socket.gethostbyname(args.host),args.port)
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0',0));self.sock.settimeout(.05)
        self.sequence=0;self.next_ping=0;self.latest=None;self.odometry=[]
        self.lock=threading.Lock()
        args.output.parent.mkdir(parents=True,exist_ok=True)
        self.log=args.output.open('w')
        self.node.subscribe(Odometry,'/model/tracking_target/odometry',self.on_odometry)

    def record(self,kind,**data):
        with self.lock:
            self.log.write(json.dumps(dict(kind=kind,wall_time=time.time(),monotonic=time.monotonic(),**data),ensure_ascii=False)+'\n');self.log.flush()
        if kind=='event':print(data,flush=True)

    def on_odometry(self,msg):
        p=msg.pose.position
        point=dict(sim_time=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,x=p.x,y=p.y,z=p.z)
        self.odometry.append(point)
        self.record('odometry',**point)

    def send(self,cmd,**kwargs):
        self.sequence+=1;message=dict(cmd=cmd,seq=self.sequence,**kwargs)
        self.sock.sendto(json.dumps(message).encode(),self.address)
        self.record('command',payload=message)

    def drive(self,speed=0,turn=0):
        msg=self.Twist();msg.linear.x=speed;msg.angular.z=turn;self.pub.publish(msg)

    def collect(self,seconds,predicate=None,*,simulation=False,speed=0,turn=0):
        deadline=time.monotonic()+(max(40,seconds*10) if simulation else seconds)
        sim_start=self.latest['ts_mono'] if self.latest else None;samples=[]
        while time.monotonic()<deadline:
            self.drive(speed,turn)
            if time.monotonic()>=self.next_ping:
                self.send('ping');self.next_ping=time.monotonic()+.5
            try:data,source=self.sock.recvfrom(65535)
            except socket.timeout:continue
            if source!=self.address:continue
            t=json.loads(data);self.record('received',payload=t)
            if t.get('type')!='telemetry':continue
            self.latest=t;samples.append(t)
            assert t['protocol_version']==8 and t['angle_convention']=='motion_head_joint_right_positive','错误的吊舱协议'
            assert all(POD_KINEMATICS.joints[a+'_joint']['limits'][0]-.2 <= t['joints_deg'][a]['pos']
                <= POD_KINEMATICS.joints[a+'_joint']['limits'][1]+.2 for a in ('yaw','pitch')),'机械限位越界'
            if predicate and predicate(t):return samples
            if simulation:
                if sim_start is None:sim_start=t['ts_mono']
                if t['ts_mono']-sim_start>=seconds:return samples
        if simulation:raise AssertionError('仿真时间未达要求')
        return samples

    def prepare(self):
        assert self.collect(40,lambda t:True),'没有收到遥测'
        self.drive();self.send('stop');set_pose('occlusion_panel',HIDDEN_PANEL)
        set_pose('tracking_target',INITIAL,INITIAL_POSE[3:])
        self.send('reset')
        zero=lambda t:t['mode']=='IDLE' and all(
            abs(t['joints_deg'][a]['pos'])<.3 and abs(t['joints_deg'][a]['vel'])<.2
            for a in ('yaw','pitch'))
        samples=self.collect(25,zero)
        assert samples and zero(samples[-1]),'两轴回零未到位'
        yaw,pitch=initial_angles()
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            t=self.latest;j=t['joints_deg'];ep=pitch-j['pitch']['pos'];ey=yaw-j['yaw']['pos']
            if abs(ep)<.2 and abs(ey)<.2:
                self.send('stop');self.collect(.3);self.record('event',event='prepared',pitch_deg=pitch);return
            self.send('manual',yaw=max(-20,min(20,4*ey)),pitch=max(-25,min(25,4*ep)))
            assert self.collect(.1),'初始化遥测中断'
        raise AssertionError('相机未对准空中靶标')

    def start_tracking(self):
        self.collect(.5)
        self.send('set_vision_mode',mode='manual_track')
        self.collect(.2)
        self.send('track',box=self.args.box)
        t=self.collect(8,tracked)
        assert t and tracked(t[-1]),'选框未进入TRACK，请检查视频/选框'
        self.collect(1,simulation=True)

    def moving(self):
        self.prepare();self.start_tracking();start=len(self.odometry);samples=[]
        for name,speed,turn in [('forward',self.args.speed,0),('reverse',-self.args.speed,0),
            ('curve',self.args.speed,.06),('curve_reverse',-self.args.speed,-.06)]:
            self.record('event',event='segment',name=name,speed=speed,turn=turn)
            samples+=self.collect(self.args.leg_seconds,simulation=True,speed=speed,turn=turn)
        self.drive();self.collect(.4)
        points=self.odometry[start:];assert points,'没有动态靶标里程计'
        distance=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(points,points[1:]))
        valid=[t for t in samples if tracked(t)]
        center=percentile([float(t['stability']['center_err_deg']) for t in valid])
        truth=percentile([truth_error_deg(t,self.odometry) for t in valid])
        ratio=len(valid)/len(samples) if samples else 0
        height=max(p['z'] for p in points)-min(p['z'] for p in points)
        result=dict(track_ratio=ratio,p95_center_error_deg=center,p95_truth_error_deg=truth,actual_distance_m=distance,height_range_m=height,telemetry_samples=len(samples),sim_seconds=samples[-1]['ts_mono']-samples[0]['ts_mono'])
        self.record('event',event='moving_metrics',**result)
        assert distance>=self.args.speed*self.args.leg_seconds*2,'实际运动距离不足'
        assert height<.1,'无重力靶标高度异常'
        assert ratio>=.95,f'TRACK比例不足 {ratio:.1%}'
        assert center<=3,f'中心误差P95超标 {center:.2f}°'
        assert truth<=3,f'靶标真值误差P95超标 {truth:.2f}°'
        self.record('event',event='moving_pass')
        return result

    def occlusion(self):
        self.prepare();self.start_tracking()
        cycles=[]
        with ThreadPoolExecutor(max_workers=1) as pool:
            for cycle in (1,2):
                samples=[]
                future=pool.submit(set_pose,'occlusion_panel',(15.,0.,60.))
                while not future.done():samples+=self.collect(.1)
                future.result();self.record('event',event='occlusion_confirmed',cycle=cycle)
                samples+=self.collect(2.)
                future=pool.submit(set_pose,'occlusion_panel',HIDDEN_PANEL)
                while not future.done():samples+=self.collect(.1)
                future.result();self.record('event',event='clear_confirmed',cycle=cycle)
                recovered=self.collect(5,tracked)
                states={t['tracking']['state'] for t in samples+recovered}
                assert 'TEMP_LOST' in {t['tracking']['state'] for t in samples},f'遮挡{cycle}未检出丢失: {states}'
                assert 'LOST' not in states,f'遮挡{cycle}进入终止LOST'
                assert recovered and tracked(recovered[-1]),f'遮挡{cycle}未恢复'
                stable=self.collect(1.)
                assert stable and all(tracked(t) for t in stable),f'遮挡{cycle}恢复后不稳定'
                cycles.append(dict(cycle=cycle,states=sorted(states)))
                self.record('event',event='occlusion_cycle_pass',**cycles[-1])
        return cycles

    def close(self):
        self.drive()
        try:set_pose('occlusion_panel',HIDDEN_PANEL)
        finally:
            self.send('stop');self.send('bye');self.sock.close()
            self.node.unsubscribe('/model/tracking_target/odometry')
            self.log.close()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host',default='127.0.0.1');ap.add_argument('--port',type=int,default=8766)
    ap.add_argument('--prepare-only',action='store_true')
    ap.add_argument('--observe-seconds',type=float,default=0,help='启动地面站观察移动靶标，保存界面截图；与验收测试分开运行')
    ap.add_argument('--test',choices=['all','moving','occlusion'],default='all')
    ap.add_argument('--speed',type=float,default=.5);ap.add_argument('--leg-seconds',type=float,default=10)
    ap.add_argument('--box',type=float,nargs=4,default=[.45,.47,.10,.06])
    ap.add_argument('--output',type=Path,default=ROOT/'runs/geographic_two_axis_pod/target.jsonl')
    args=ap.parse_args()
    if not 0<args.speed<=1 or not 5<=args.leg_seconds<=20:ap.error('速度(0,1]m/s，每段[5,20]仿真秒')
    if args.observe_seconds and not 5<=args.observe_seconds<=60:ap.error('观察时间[5,60]墙钟秒')
    os.environ.setdefault('GZ_PARTITION',WORLD)
    args.output.with_suffix('.summary.json').unlink(missing_ok=True)
    session=Session(args);report={}
    try:
        session.record('event',event='start',speed=args.speed,leg_seconds=args.leg_seconds)
        if args.observe_seconds:
            session.prepare();session.start_tracking()
            snapshot=args.output.parent/'ground_station.png'
            snapshot.unlink(missing_ok=True)
            command=['bash',
                str(ROOT/'models/gimbals/two_axis_pod/ground_station/run.sh'),
                '--udp-host',args.host,'--udp-port',str(args.port),
                '--duration',str(args.observe_seconds),
                '--gui-snapshot',str(snapshot)]
            gui_log=(args.output.parent/'ground_station.log').open('w')
            gui=subprocess.Popen(command,stdout=gui_log,stderr=subprocess.STDOUT)
            try:
                session.collect(max(2,args.observe_seconds-3),speed=args.speed,turn=.04)
                session.collect(3)
                assert gui.wait(timeout=15)==0,'地面站窗口运行失败'
                assert snapshot.exists(),'地面站未保存界面截图'
                report['ground_station_snapshot']=str(snapshot)
            finally:
                if gui.poll() is None:
                    gui.terminate();gui.wait(timeout=5)
                gui_log.close()
        elif args.prepare_only:session.prepare()
        else:
            if args.test in ('all','moving'):report['moving']=session.moving()
            if args.test in ('all','occlusion'):report['occlusion']=session.occlusion()
        report['result']='passed';session.record('event',event='pass',**report)
        args.output.with_suffix('.summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    except BaseException as error:
        session.record('event',event='failure',error=str(error));raise
    finally:session.close()


if __name__=='__main__':main()
