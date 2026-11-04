#!/usr/bin/env python3
"""核对已有会话真实CameraInfo和演示标靶渲染，独立投影已知几何，不重绘图像。"""
from __future__ import annotations
import argparse
import itertools
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.session import load_session
from tools.check_physics import Probe


def rx(a):
    c,s=math.cos(a),math.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])


def ry(a):
    c,s=math.cos(a),math.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])


def rz(a):
    c,s=math.cos(a),math.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def transform(pose):
    p=np.fromstring(pose or '0 0 0 0 0 0',sep=' ')
    if p.shape!=(6,):raise ValueError(f'非法夹具pose：{pose}')
    return p[:3],rz(p[5])@ry(p[4])@rx(p[3])


def camera_pose(model,joints):
    position,mount=transform(model.findtext('pose'))
    yaw_link=model.find("link[@name='yaw_arm_link']")
    camera_link=model.find("link[@name='camera_link']")
    pivot,_=transform(yaw_link.findtext('pose'))
    center,_=transform(camera_link.findtext('pose'))
    offset,sensor=transform(camera_link.findtext('sensor/pose'))
    yaw=math.radians(joints['yaw']['pos']);pitch=math.radians(joints['pitch']['pos'])
    chain=rx(yaw)@ry(-pitch)
    optical_position=position+mount@(pivot+rx(yaw)@(center-pivot)+chain@offset)
    return optical_position,mount@chain@sensor


def expected_target(target,camera_position,camera_rotation,k,width,height):
    position,rotation=transform(target.findtext('pose'))
    points=[]
    for visual in target.findall('link/visual'):
        size=visual.findtext('geometry/box/size')
        if size is None:continue
        local,orientation=transform(visual.findtext('pose'))
        half=np.fromstring(size,sep=' ')/2
        for signs in itertools.product((-1,1),repeat=3):
            point=position+rotation@(local+orientation@(half*np.asarray(signs)))
            p=camera_rotation.T@(point-camera_position)
            if p[0]<=0:raise AssertionError('光学标靶位于相机后方')
            points.append((k[2]-k[0]*p[1]/p[0],k[5]-k[4]*p[2]/p[0]))
    if not points:raise AssertionError('夹具缺少已知box几何')
    p=np.asarray(points)
    low=np.maximum(p.min(axis=0),[0,0]);high=np.minimum(p.max(axis=0),[width,height])
    if np.any(high<=low):return None
    center=camera_rotation.T@(position-camera_position)
    return {'bounds_px':[float(low[0]),float(low[1]),float(high[0]),float(high[1])],
            'width_px':float(high[0]-low[0]),'height_px':float(high[1]-low[1]),
            'center_px':[float(k[2]-k[0]*center[1]/center[0]),float(k[5]-k[4]*center[2]/center[0])]}


def color_mask(rgb,index):
    r,g,b=[rgb[:,:,i].astype(float) for i in range(3)]
    if index%3==0:return (r>1.65*g)&(r>1.65*b)&(r>45)
    if index%3==1:return (b>1.4*r)&(b>1.35*g)&(b>45)
    return (r>1.7*b)&(g>1.7*b)&(g>.5*r)&(r>45)


def run(probe,session,tolerance):
    probe.send(0)
    probe.wait(lambda v:v.get('mechanical_valid') and v.get('config_hash')==probe.c['config_hash'],description='统一配置和真实机械数据')
    probe.home('光学验收回零')
    probe.wait(lambda _:bool(probe.camera_info),description='真实CameraInfo')
    started=time.monotonic()
    probe.wait(lambda _:probe.image is not None and probe.image[0]>started,description='真实相机新图像')
    with probe.lock:
        info=dict(probe.camera_info);_,width,height,stride,pixel_format,data=probe.image
        joints=dict(probe.latest['joints_deg'])
    k=info['k'];derived=probe.c['derived']
    if len(k)!=9:raise AssertionError(f'CameraInfo内参不是3×3：{k}')
    for index,key in [(0,'fx'),(4,'fy'),(2,'cx'),(5,'cy')]:
        if not math.isclose(k[index],derived[key],rel_tol=1e-7,abs_tol=1e-5):
            raise AssertionError(f'实际CameraInfo {key}={k[index]}与配置{derived[key]}不符')
    if abs(derived['fx']-derived['fy'])>1e-5 and abs(k[0]-k[4])<1e-5:
        raise AssertionError('CameraInfo把独立fy替换成fx')
    expected_size=(probe.c['optics']['width_px'],probe.c['optics']['height_px'])
    if (width,height)!=expected_size or (info['width'],info['height'])!=expected_size:
        raise AssertionError(f'实际图像/CameraInfo分辨率不符：{width}×{height}')
    if stride<width*3 or len(data)<stride*height:raise AssertionError('真实相机图像不是完整RGB8数据')
    raw=Image.frombytes('RGB',(width,height),data,'raw','RGB',stride,1)
    image_path=probe.output/'actual_render.png';raw.save(image_path)
    rgb=np.asarray(raw)
    world_path=Path(session['world_path'])
    world=ET.parse(world_path).getroot().find('world')
    model=world.find(f"model[@name='{session['model']}']")
    if model is None:raise AssertionError('实际world缺少本中科模型')
    position,rotation=camera_pose(model,joints)
    comparisons=[]
    for index,name in enumerate(probe.c['tracking']['target_names']):
        target=world.find(f"model[@name='{name}']")
        if target is None:raise AssertionError(f'实际world缺少光学标靶{name}')
        expected=expected_target(target,position,rotation,k,width,height)
        if expected is None:continue
        # 在已知投影周围检查同色真实像素；不以真值候选或离线框充当渲染检测。
        mask=color_mask(rgb,index)
        left,top,right,bottom=expected['bounds_px']
        x0=max(0,int(math.floor(left))-8);x1=min(width,int(math.ceil(right))+8)
        y0=max(0,int(math.floor(top))-8);y1=min(height,int(math.ceil(bottom))+8)
        ys,xs=np.where(mask[y0:y1,x0:x1])
        if len(xs)<10:raise AssertionError(f'{name}真实图像中缺少已知颜色标靶像素')
        observed=[int(xs.min()+x0),int(ys.min()+y0),int(xs.max()+x0+1),int(ys.max()+y0+1)]
        errors=[float(a-b) for a,b in zip(observed,expected['bounds_px'])]
        observed_size=[observed[2]-observed[0],observed[3]-observed[1]]
        comparison={'name':name,'expected':expected,'observed_bounds_px':observed,
                    'observed_size_px':observed_size,'bounds_error_px':errors,'color_pixel_count':int(len(xs))}
        comparisons.append(comparison)
        if max(abs(e) for e in errors)>tolerance:
            raise AssertionError(f'{name}实际渲染边界与独立投影不符：{comparison}')
    if not comparisons:raise AssertionError('没有完整或部分入画的已知光学标靶')
    return {'result':'passed','config_hash':probe.c['config_hash'],
            'focal_length_mm':probe.c['optics']['focal_length_mm'],
            'sensor_width_mm':derived['sensor_width_mm'],'sensor_height_mm':derived['sensor_height_mm'],
            'actual_hfov_deg':derived['hfov_deg'],'actual_vfov_deg':derived['vfov_deg'],
            'camera_info':info,'real_image':str(image_path),'pixel_format':pixel_format,
            'camera_world_position_m':position.tolist(),'joints_deg':joints,
            'raster_bounds_tolerance_px':tolerance,'targets':comparisons,
            'method':'真实RGB图像颜色像素边界，对照实际world中box八角点的独立透视投影；无离线重绘'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',default='validation');parser.add_argument('--output',type=Path)
    parser.add_argument('--pixel-tolerance',type=float,default=3.)
    args=parser.parse_args();session=load_session(args.session)
    output=args.output or Path(session['session_path']).parent/'optics'
    probe=Probe(session,output)
    try:
        result=run(probe,session,args.pixel_tolerance);print('中科真实CameraInfo和光学渲染检查通过')
    except Exception as exc:
        result={'result':'failed','error':str(exc),'config_hash':probe.c['config_hash']}
        print(f'中科光学检查失败：{exc}',file=sys.stderr)
    finally:probe.close()
    (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return 0 if result['result']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
