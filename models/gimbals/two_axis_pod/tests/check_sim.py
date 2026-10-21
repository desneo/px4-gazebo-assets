#!/usr/bin/env python3
"""在线检查实际Gazebo关节、机械限位、回零、光学帧和跟踪。仿真需预先启动。"""
import argparse
import json
import math
from pathlib import Path
import threading
import time
from gz.msgs10.image_pb2 import Image
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node
from PIL import Image as PILImage


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',type=Path,default=Path('runs/two_axis_pod'))
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    node=Node(); lock=threading.Lock(); tel={}; results={}; frames={}
    def telemetry(m):
        with lock: tel.clear();tel.update(json.loads(m.data))
    def result(m):
        r=json.loads(m.data)
        with lock: results[r.get('request_id')]=r
    def image(kind):
        def cb(m):
            with lock: frames[kind]=(m.width,m.height,bytes(m.data))
        return cb
    node.subscribe(StringMsg,'/two_axis_pod/telemetry',telemetry)
    node.subscribe(StringMsg,'/two_axis_pod/command_result',result)
    callbacks=[image('camera'),image('overview')]
    node.subscribe(Image,'/world/two_axis_pod_demo/model/two_axis_pod/link/camera_link/sensor/camera/image',callbacks[0])
    node.subscribe(Image,'/two_axis_pod/overview',callbacks[1])
    pub=node.advertise('/two_axis_pod/command',StringMsg)
    counter=0
    def send(cmd,**kwargs):
        nonlocal counter
        counter+=1
        key=str(counter); m=StringMsg(); m.data=json.dumps(dict(cmd=cmd,request_id=key,**kwargs));pub.publish(m)
        return key
    def snapshot():
        with lock:return json.loads(json.dumps(tel))
    def wait(predicate,seconds=12):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            value=snapshot()
            if value and predicate(value):return value
            time.sleep(.04)
        raise AssertionError('状态等待超时: '+str(snapshot()))
    def accepted(cmd,expected=True,**kwargs):
        key=send(cmd,**kwargs);deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            with lock:r=results.get(key)
            if r is not None:
                assert r['ok']==expected,r
                return r
            time.sleep(.02)
        raise AssertionError('缺少指令回执 '+cmd)
    def home():
        accepted('home')
        return wait(lambda t:t['mode']=='IDLE' and all(
            abs(t['joints_deg'][a]['pos'])<.15 and abs(t['joints_deg'][a]['vel'])<.2
            for a in ('yaw','pitch')),15)
    def drive(axis,rate,target):
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            send('manual',**{axis:rate})
            t=snapshot();pos=t['joints_deg'][axis]['pos']
            assert -90.2<=t['joints_deg']['yaw']['pos']<=90.2,t
            assert -90.2<=t['joints_deg']['pitch']['pos']<=45.2,t
            if abs(pos-target)<.4 and t['limits'][axis]:
                accepted('stop');return pos
            time.sleep(.1)
        raise AssertionError(f'{axis}未到限位 {target}: {snapshot()}')
    t=wait(lambda t:'joints_deg' in t)
    assert t['protocol_version']==5 and t['angle_convention']=='upright_joint'
    assert set(t['joints_deg'])=={'yaw','pitch'}
    accepted('reset_axis',expected=False,axis='roll')
    accepted('manual',expected=False,roll=0)
    home()
    # 保存实际渲染画面；非示意图。
    def rendered(frame):
        data=frame[2]
        step=max(1,len(data)//4096)
        return len(set(data[::step]))>3
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        with lock:ready=len(frames)==2 and all(rendered(frame) for frame in frames.values())
        if ready:break
        time.sleep(.1)
    with lock: saved=dict(frames)
    assert len(saved)==2 and all(rendered(frame) for frame in saved.values()),'相机未完成有效渲染'
    for name,(w,h,data) in saved.items():
        PILImage.frombytes('RGB',(w,h),data).save(args.output/f'{name}_zero.png')
    accepted('track',box=[.41,.34,.18,.32])
    t=wait(lambda t:t['tracking']['state']=='TRACK' and t['target']['valid'],8)
    assert t['tracking']['valid_frames']>0
    accepted('stop');home()
    t=snapshot();assert t['singular'] and t['los_mount'][2]>.999
    for _ in range(6):send('manual',yaw=30);time.sleep(.1)
    accepted('stop');t=snapshot()
    assert t['joints_deg']['yaw']['pos']>2 and t['los_mount'][2]>.999
    home()
    positions={}
    for axis,rate,target in [('yaw',80,90),('yaw',-80,-90),('pitch',-80,-90),('pitch',80,45)]:
        home();positions[f'{axis}_{target}']=drive(axis,rate,target)
        if axis=='pitch' and target==-90:
            t=snapshot();assert not t['singular'] and t['los_mount'][0]>.999
    home();send('manual',yaw=20,pitch=20)
    wait(lambda t:t['mode']=='MANUAL')
    t=wait(lambda t:t['mode']=='IDLE' and all(abs(t['cmd_dps'][a])<.001 for a in ('yaw','pitch')),3)
    accepted('reset_axis',axis='yaw')
    wait(lambda t:abs(t['joints_deg']['yaw']['pos'])<.15)
    assert abs(snapshot()['joints_deg']['pitch']['pos']-t['joints_deg']['pitch']['pos'])<.2
    home();accepted('stop')
    report={'result':'passed','actual_limits_deg':positions,'rendered_frames':list(saved),'tracking':'CSRT TRACK valid','watchdog':'passed','single_axis_reset':'passed','skyward_singularity_manual':'passed'}
    (args.output/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':main()
