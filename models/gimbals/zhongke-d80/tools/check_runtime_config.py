#!/usr/bin/env python3
"""只读查询Gazebo当前世界，核对实际加载的中科质量/惯性、关节与相机配置。"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.session import load_session
from common.config import load_config


def finite(text,label):
    if text is None:raise AssertionError(f'运行世界缺少{label}，不能证明实际生效')
    result=float(text)
    if not math.isfinite(result):raise AssertionError(f'运行世界{label}不是有限值')
    return result


def compare(actual,expected,label,relative=1e-10,absolute=1e-12):
    if not math.isfinite(actual) or not math.isclose(actual,expected,rel_tol=relative,abs_tol=absolute):
        raise AssertionError(f'{label}不一致：运行值={actual}，配置预期={expected}')


def check(session,output,timeout):
    c=load_config(session['config_path'])
    if c['config_hash']!=session['config_hash']:
        raise AssertionError('当前INI与运行清单hash不一致，请先按改参流程重启')
    os.environ['GZ_PARTITION']=session['gz_partition']
    from gz.transport13 import Node
    from gz.msgs10.sdf_generator_config_pb2 import SdfGeneratorConfig
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.camera_info_pb2 import CameraInfo
    node=Node();lock=threading.Lock();live={}
    prefix=session.get('topic_prefix','/zhongke_d80')
    def on_telemetry(msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        with lock:live['telemetry']=data;live['telemetry_arrival']=time.monotonic()
    def on_camera(msg):
        with lock:
            live['camera_info']={'width':msg.width,'height':msg.height,'k':list(msg.intrinsics.k),'p':list(msg.projection.p)}
            live['camera_arrival']=time.monotonic()
    request=SdfGeneratorConfig()
    request.global_entity_gen_config.expand_include_tags.data=True
    service='/world/'+session['world']+'/generate_world_sdf'
    ok,response=node.request(service,request,SdfGeneratorConfig,StringMsg,int(timeout*1000))
    if not ok or not response.data.strip():
        raise AssertionError(f'无法从运行世界服务{service}取得实际质量/惯性；不使用启动前SDF替代')
    raw=response.data
    node.subscribe(StringMsg,prefix+'/telemetry',on_telemetry)
    node.subscribe(CameraInfo,session.get('camera_info_topic',prefix+'/camera_info'),on_camera)
    output.parent.mkdir(parents=True,exist_ok=True)
    export_path=output.parent/'runtime_world.sdf';export_path.write_text(raw)
    world=ET.fromstring(raw).find('world')
    if world is None or world.get('name')!=session['world']:raise AssertionError('运行服务返回世界身份不一致')
    models=[m for m in world.findall('model') if m.get('name')==session['model']]
    if len(models)!=1:raise AssertionError('运行世界未提供唯一的中科模型，不能核对实际加载参数')
    model=models[0]
    properties=json.loads((ROOT/'cad/properties.json').read_text())
    links=[];total=0
    for name,group in properties['groups'].items():
        link=model.find(f"link[@name='{name}']")
        if link is None:raise AssertionError(f'运行世界缺少实际刚体{name}')
        actual_mass=finite(link.findtext('inertial/mass'),name+'.mass')
        expected_mass=c['model']['mass_kg']*group['volume_mm3']/properties['volume_mm3']
        compare(actual_mass,expected_mass,name+'.mass');total+=actual_mass
        tensors={}
        for field,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:
            actual=finite(link.findtext('inertial/inertia/'+field),name+'.'+field)
            expected=expected_mass*group['unit_mass_inertia_m2'][i][j]
            compare(actual,expected,name+'.'+field,absolute=1e-15)
            tensors[field]=actual
        pose_text=link.findtext('inertial/pose')
        if pose_text is None:raise AssertionError(f'运行世界缺少{name}局部质心')
        pose=[float(x) for x in pose_text.split()]
        if len(pose)!=6:raise AssertionError('运行惯性pose不是6维')
        for i in range(3):compare(pose[i],group['center_local_m'][i],name+'.center_'+str(i))
        links.append({'name':name,'runtime_mass_kg':actual_mass,'expected_mass_kg':expected_mass,
                      'runtime_inertia_kg_m2':tensors,'runtime_center_local_m':pose[:3]})
    compare(total,c['model']['mass_kg'],'runtime.total_mass')
    joints=[]
    for axis in ('yaw','pitch'):
        joint=model.find(f"joint[@name='{axis}_joint']")
        if joint is None:raise AssertionError(f'运行世界缺少{axis}关节')
        actual={}
        for field,expected in [('lower',math.radians(c['mechanics'][axis+'_min_deg'])),
                               ('upper',math.radians(c['mechanics'][axis+'_max_deg'])),
                               ('velocity',math.radians(c['mechanics']['max_rate_dps']))]:
            value=finite(joint.findtext('axis/limit/'+field),axis+'.'+field)
            compare(value,expected,axis+'.'+field)
            actual[field]=value
        joints.append({'name':axis+'_joint','runtime_limit_rad_and_rad_s':actual,
                       'runtime_limit_deg_and_dps':{key:math.degrees(value) for key,value in actual.items()}})
    camera=model.find("link[@name='camera_link']/sensor[@name='camera']/camera")
    if camera is None:raise AssertionError('运行世界缺少实际相机配置')
    runtime_camera={}
    for key,config_key in [('width','width_px'),('height','height_px')]:
        value=finite(camera.findtext('image/'+key),'camera.'+key)
        compare(value,c['optics'][config_key],'camera.'+key)
        runtime_camera[key]=int(value)
    for key in ('fx','fy','cx','cy'):
        value=finite(camera.findtext('lens/intrinsics/'+key),'camera.'+key)
        compare(value,c['derived'][key],'camera.'+key,absolute=1e-7)
        runtime_camera[key]=value
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        with lock:ready='telemetry' in live and 'camera_info' in live
        if ready:break
        time.sleep(.02)
    with lock:snapshot=dict(live)
    if 'telemetry' not in snapshot or 'camera_info' not in snapshot:
        raise AssertionError('运行SDF已取得，但缺少新鲜插件遥测/真实CameraInfo，核验不完整')
    telemetry=snapshot['telemetry'];info=snapshot['camera_info']
    if telemetry.get('config_hash')!=c['config_hash']:raise AssertionError('实际插件的配置hash与当前会话不一致')
    if telemetry.get('model_name')!=session['model']:raise AssertionError('实际插件模型身份不一致')
    for key in ('fx','fy','cx','cy'):
        compare(finite(str(telemetry.get('optics',{}).get(key)), 'plugin.'+key),c['derived'][key],'plugin.'+key,absolute=1e-7)
    if len(info['k'])!=9:raise AssertionError('真实CameraInfo缺少3×3 K')
    for index,key in [(0,'fx'),(4,'fy'),(2,'cx'),(5,'cy')]:
        compare(info['k'][index],c['derived'][key],'CameraInfo.'+key,relative=1e-7,absolute=1e-5)
    if (info['width'],info['height'])!=(c['optics']['width_px'],c['optics']['height_px']):
        raise AssertionError('真实CameraInfo分辨率与当前会话不一致')
    return {'result':'passed','session_path':session['session_path'],'config_hash':c['config_hash'],
            'queried_unix_ms':int(time.time()*1000),'source_service':service,
            'runtime_export':str(export_path),'runtime_export_sha256':hashlib.sha256(raw.encode()).hexdigest(),
            'runtime_total_mass_kg':total,'links':links,'joints':joints,'runtime_camera':runtime_camera,
            'camera_info':info,'plugin_config_hash':telemetry['config_hash'],
            'method':'从已运行Gazebo的generate_world_sdf服务取得当前加载模型，并结合新鲜插件遥测与真实CameraInfo；未重读启动前SDF作为实际值',
            'control_actions':'无；只读Gazebo服务和订阅，不建立UDP控制租约'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',default='validation');parser.add_argument('--output',type=Path)
    parser.add_argument('--timeout',type=float,default=8.)
    args=parser.parse_args();session=load_session(args.session)
    output=args.output or Path(session['session_path']).parent/'runtime_config.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    try:
        result=check(session,output,args.timeout);print('中科运行世界质量/惯性、关节和相机配置核对通过')
    except Exception as exc:
        result={'result':'failed','error':str(exc),'session_path':session['session_path'],
                'control_actions':'无；没有向UDP或Gazebo发送运动控制'}
        print(f'中科运行配置核对失败：{exc}',file=sys.stderr)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return 0 if result['result']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
