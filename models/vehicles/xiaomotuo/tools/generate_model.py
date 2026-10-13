#!/usr/bin/env python3
"""在 PX4 advanced_plane 原生接口基础上生成通用固定翼小摩托的装饰三角翼外形。"""
import argparse, copy, json, math, shutil, re
from pathlib import Path
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
def setv(el,path,value):
    node=el.find(path)
    if node is None:
        node=el
        for part in path.split('/'):
            child=node.find(part)
            node=child if child is not None else E.SubElement(node,part)
    node.text=str(value)
    return node

def box(parent,name,xyz,size,color,collision=False,rpy=(0,0,0)):
    v=E.SubElement(parent,'collision' if collision else 'visual',name=name)
    setv(v,'pose',' '.join(map(str,(*xyz,*rpy))))
    setv(E.SubElement(E.SubElement(v,'geometry'),'box'),'size',' '.join(map(str,size)))
    if not collision:
        mat=E.SubElement(v,'material'); setv(mat,'ambient',color); setv(mat,'diffuse',color)
    return v

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,default=ROOT/'parameters.json');ap.add_argument('--output-dir',type=Path,default=ROOT);a=ap.parse_args()
    p=json.loads(a.config.read_text());out=a.output_dir;out.mkdir(parents=True,exist_ok=True);(out/'meshes').mkdir(exist_ok=True)
    for key in ('length_m','span_m','mass_kg','wing_area_m2','mean_chord_m','target_airspeed_m_s','propeller_diameter_m','motor_max_rad_s','motor_constant','wheel_axle_friction_nm'):
        if not isinstance(p[key],(int,float)) or not math.isfinite(p[key]) or p[key]<=0:raise ValueError(key+' 必须为有限正数')
    if p['mass_kg']<=1:raise ValueError('总质量必须大于辅助部件1kg')
    tree=E.parse(ROOT/'tools/px4_advanced_plane.sdf');model=tree.getroot().find('model');model.set('name','xiaomotuo');setv(model,'pose','0 0 0.57 0 0 0')
    # 保留上游传感器和控制系统；替换所有旧几何与关节位置。
    for n in list(model):
        if n.tag=='include' or n.get('name','').startswith('lidar_'):model.remove(n)
    wheel_template=E.fromstring('<link><pose relative_to="base_link">0 0 0 0 0 0</pose><inertial><mass>1</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial></link>')
    for name in ('LeftWheel','RightWheel','CenterWheel'):
        l=copy.deepcopy(wheel_template);l.set('name',name);model.append(l)
        j=E.SubElement(model,'joint',name=name+'Joint',type='revolute');setv(j,'parent','base_link');setv(j,'child',name);setv(j,'axis/xyz','0 1 0');setv(j,'axis/dynamics/friction',p['wheel_axle_friction_nm']);setv(l,'pose','0 0 0 0 0 0');l.find('pose').set('relative_to',name+'Joint')
    for l in model.findall('link'):
        for n in list(l):
            if n.tag in ('visual','collision'): l.remove(n)
    base=model.find("link[@name='base_link']");setv(base,'pose','0 0 0 0 0 0')
    setv(base,'inertial/mass',p['mass_kg']-1.0);setv(base,'inertial/pose','0 0 0 0 0 0')
    for key,val in zip(('ixx','iyy','izz'),p['inertia_kg_m2']):setv(base,'inertial/inertia/'+key,val)
    # 总长从机头 +1.55 到尾翼后缘 -1.75，翼尖 ±1.25。
    white='0.88 0.9 0.94 1';blue='0.02 0.24 0.72 1';dark='0.03 0.04 0.06 1'
    box(base,'body_collision',(.45,0,0),(2.2,.35,.35),white,True)
    box(base,'wing_collision',(0,0,.12),(.65,2.5,.065),white,True)
    for y in (-.72,.72):
        box(base,'boom_collision_'+str(y),(-.75,y,.05),(1.9,.065,.075),dark,True)
        box(base,'fin_collision_'+str(y),(-1.48,y,.30),(.45,.035,.5),blue,True)
    box(base,'tailplane_collision',(-1.50,0,.18),(.5,1.55,.045),white,True)
    positions={'left_elevon':(-.22,.86,.12),'right_elevon':(-.22,-.86,.12),'elevator':(-1.62,0,.18),'rudder':(-1.62,.72,.32)}
    for linkname,pose in positions.items():
        l=model.find(f"link[@name='{linkname}']");setv(l,'pose',' '.join(map(str,(*pose,0,0,0))));setv(l,'inertial/pose','0 0 0 0 0 0');setv(l,'inertial/mass',.05)
        for k in ('ixx','iyy','izz'):setv(l,'inertial/inertia/'+k,.0005)
        size=(.2,.7,.035) if 'elevon' in linkname else ((.20,1.4,.035) if linkname=='elevator' else (.2,.035,.35))
        joint=model.find(f"joint[child='{linkname}']");setv(joint,'pose','0 0 0 0 0 0')
    # 未使用襟翼通道保留原生接口与零体积辅助 link。
    for name in ('left_flap','right_flap'):
        l=model.find(f"link[@name='{name}']");setv(l,'inertial/mass',.000001)
    prop=model.find("link[@name='rotor_puller']");setv(prop,'pose','-.72 0 .10 0 1.57079632679 0');setv(prop,'inertial/mass',.3)
    for name,x,y in [('LeftWheel',-.18,.48),('RightWheel',-.18,-.48),('CenterWheel',1.,0)]:
        l=model.find(f"link[@name='{name}']");setv(l,'inertial/mass',(.5-.000002)/3)
        for k in ('ixx','iyy','izz'):setv(l,'inertial/inertia/'+k,.001)
        j=model.find(f"joint[child='{name}']");setv(j,'pose',f'{x} {y} -.40 0 0 0');j.find('pose').set('relative_to','base_link')
        for typ in ('visual','collision'):
            n=E.SubElement(l,typ,name=name+'_'+typ);setv(n,'pose','0 0 0 1.57079632679 0 0');c=E.SubElement(E.SubElement(n,'geometry'),'cylinder');setv(c,'radius',.15);setv(c,'length',.08)
            if typ=='visual':mat=E.SubElement(n,'material');setv(mat,'ambient',dark);setv(mat,'diffuse',dark)
            else:
                friction=E.SubElement(E.SubElement(E.SubElement(n,'surface'),'friction'),'ode');setv(friction,'mu',.9);setv(friction,'mu2',.9)
    aero=model.find("plugin[@name='gz::sim::systems::AdvancedLiftDrag']")
    for k,v in p['aero'].items():setv(aero,k,v)
    for k,v in {'area':p['wing_area_m2'],'mac':p['mean_chord_m'],'AR':p['span_m']**2/p['wing_area_m2'],'cp':'0 0 0'}.items():setv(aero,k,v)
    motor=model.find("plugin[@name='gz::sim::systems::MulticopterMotorModel']")
    for k,v in {'maxRotVelocity':p['motor_max_rad_s'],'motorConstant':p['motor_constant'],'momentConstant':.001,'rotorDragCoefficient':0,'rollingMomentCoefficient':0}.items():setv(motor,k,v)
    # 尺寸修改同步作用于结构几何和关节，不缩放传感器噪声或质量。
    sx=p['length_m']/3.3;sy=p['span_m']/2.5
    for n in model.iter('pose'):
        values=list(map(float,n.text.split()));values[0]*=sx;values[1]*=sy;n.text=' '.join(map(str,values))
    for link in model.findall('link'):
        for n in link.findall('.//box/size'):
            values=list(map(float,n.text.split()));values[0]*=sx
            if link.get('name')!='rotor_puller':values[1]*=sy
            n.text=' '.join(map(str,values))
    # 外形为纯美术换壳；全部通用物理、控制、传感器节点保持不变。
    for link in model.findall('link'):
        for visual in list(link.findall('visual')):
            if link.get('name') not in ('LeftWheel','RightWheel','CenterWheel'):
                link.remove(visual)
    artwork=E.parse(ROOT/'tools/artwork.sdf').getroot().find('model/link')
    for original in artwork.findall('visual'):
        visual=copy.deepcopy(original)
        uri=visual.find('geometry/mesh/uri');mesh_name=Path(uri.text).name
        uri.text='model://xiaomotuo/meshes/'+mesh_name
        setv(visual.find('geometry/mesh'),'scale',f'{sx} {sy} 1')
        if original.get('name')=='decorative_propeller':
            setv(visual,'pose',f'.025 0 {.72*sx} 0 -1.57079632679 0');prop.append(visual)
        else:
            setv(visual,'pose','0 0 .075 0 0 0');base.append(visual)
        if out.resolve()!=ROOT.resolve():shutil.copy2(ROOT/'meshes'/mesh_name,out/'meshes'/mesh_name)
    for name,x,y in [('nose_gear',1.0,0),('left_gear',-.18,.48),('right_gear',-.18,-.48)]:
        visual=E.SubElement(base,'visual',name=name)
        setv(visual,'pose',f'{x*sx} {y*sy} -.22 0 0 0')
        cylinder=E.SubElement(E.SubElement(visual,'geometry'),'cylinder');setv(cylinder,'radius',.024);setv(cylinder,'length',.38)
        mat=E.SubElement(visual,'material');setv(mat,'ambient','.12 .14 .16 1');setv(mat,'diffuse','.12 .14 .16 1')
    E.indent(tree,space='  ');tree.write(out/'model.sdf',encoding='utf-8',xml_declaration=True)
    (out/'model.config').write_text('<?xml version="1.0"?><model><name>小摩托</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><author><name>px4-gazebo-assets</name></author><description>Generic fixed-wing simulator with decorative delta exterior</description></model>\n')
    if out.resolve()!=ROOT.resolve():
        shutil.copy2(a.config,out/'parameters.json')
        for folder in ('px4','tests'):shutil.copytree(ROOT/folder,out/folder,dirs_exist_ok=True)
        (out/'licenses').mkdir(exist_ok=True)
        for name in ('PX4_LICENSE','PX4_GAZEBO_LICENSE'):shutil.copy2(ROOT/'tools'/name,out/'licenses'/name)
    airframe=out/'px4/4030_gz_xiaomotuo';text=airframe.read_text();text=re.sub(r'SIM_GZ_EC_MAX1 [^\n]+','SIM_GZ_EC_MAX1 '+str(p['motor_max_rad_s']),text);
    for name,value in p.get('px4',{}).items():
        pattern=r'param set-default '+re.escape(name)+r' [^\n]+'
        replacement=f'param set-default {name} {value}'
        text=re.sub(pattern,replacement,text) if re.search(pattern,text) else text+'\n'+replacement+'\n'
    airframe.write_text(text)
    print(out/'model.sdf')
if __name__=='__main__':main()
