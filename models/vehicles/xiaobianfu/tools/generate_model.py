#!/usr/bin/env python3
"""基于 PX4 advanced_plane 系统生成参数化飞翼；原始来源见 LICENSE-PX4。"""
import argparse, copy, json, math, shutil, re
from pathlib import Path
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
def sub(parent, tag, text=None, **attr):
    e=E.SubElement(parent,tag,attr)
    if text is not None: e.text=str(text)
    return e
def setv(parent, tag, value):
    e=parent.find(tag)
    if e is None: e=sub(parent,tag)
    e.text=str(value)
def box(parent,name,pose,size,color,collision=False):
    v=sub(parent,'collision' if collision else 'visual',name=name); sub(v,'pose',pose)
    sub(sub(sub(v,'geometry'),'box'),'size',size)
    if not collision:
        m=sub(v,'material'); sub(m,'ambient',color);sub(m,'diffuse',color)
    else:
        ode=sub(sub(sub(v,'surface'),'friction'),'ode');sub(ode,'mu',0.65);sub(ode,'mu2',0.65)
    return v
def mesh(path, points, thickness):
    # 闭合凸多边形薄翼 STL，坐标米；每块翼面的弦向厚度为估计值。
    n=len(points); vertices=[(x,y,z) for z in [-thickness/2,thickness/2] for x,y in points]
    faces=[]
    for i in range(1,n-1):faces.extend([(0,i+1,i),(n,n+i,n+i+1)])
    for i in range(n): j=(i+1)%n;faces.extend([(i,j,n+j),(i,n+j,n+i)])
    with path.open('w') as f:
        f.write('solid wing\n')
        for face in faces:
            f.write('facet normal 0 0 0\nouter loop\n')
            for idx in face:f.write('vertex %g %g %g\n'%vertices[idx])
            f.write('endloop\nendfacet\n')
        f.write('endsolid wing\n')
def generate(p,out):
    positive=[p['dimensions']['length_m'],p['dimensions']['span_m'],p['aerodynamics']['area_m2'],p['aerodynamics']['mac_m'],p['motor']['max_rad_s'],p['motor']['thrust_coefficient'],p['motor']['diameter_m'],p['launch']['height_m'],p['launch']['speed_m_s']]
    if not all(math.isfinite(v) and v>0 for v in positive) or p['mass_kg']<=.016:raise ValueError('尺寸、面积、动力、手抛条件必须有限正数，质量必须大于附属链路质量0.016kg')
    inertia=p['inertia_kg_m2']
    if len(inertia)!=3 or not all(math.isfinite(v) and v>0 for v in inertia) or 2*max(inertia)>=sum(inertia):raise ValueError('主惯量必须为正且满足三角不等式')
    out.mkdir(parents=True,exist_ok=True); (out/'meshes').mkdir(exist_ok=True)
    for d in ['tests','px4','controller']:(out/d).mkdir(exist_ok=True)
    source=E.parse(ROOT/'tools/px4_advanced_plane.sdf').getroot().find('model')
    sdf=E.Element('sdf',version='1.9'); model=sub(sdf,'model',name='xiaobianfu'); base=sub(model,'link',name='base_link')
    iner=sub(base,'inertial');sub(iner,'mass',p['mass_kg']-.016)
    it=sub(iner,'inertia')
    for k,v in zip(['ixx','iyy','izz'],p['inertia_kg_m2']):sub(it,k,v)
    for k in ['ixy','ixz','iyz']:sub(it,k,0)
    for sensor in source.find("link[@name='base_link']").findall('sensor'):base.append(copy.deepcopy(sensor))
    if base.find("sensor[@type='navsat']") is None:
        s=sub(base,'sensor',name='navsat_sensor',type='navsat');sub(s,'always_on',1);sub(s,'update_rate',30)
    L=p['dimensions']['length_m']; B=p['dimensions']['span_m']; sx=L/.4; sy=B
    green='0.03 0.6 0.03 1';white='0.95 0.96 0.92 1';black='0.08 0.10 0.12 1'
    for side in [-1,1]:
        pts=[(.18*sx,side*.025*sy),(-.035*sx,side*.5*sy),(-.19*sx,side*.5*sy),(-.14*sx,side*.025*sy)]
        if side<0:pts.reverse()
        file=f'wing_{side}.stl';mesh(out/'meshes'/file,pts,.022)
        v=sub(base,'visual',name=f'wing_{side}');g=sub(v,'geometry');sub(sub(g,'mesh'),'uri',f'model://xiaobianfu/meshes/{file}')
        m=sub(v,'material');sub(m,'ambient',white);sub(m,'diffuse',white)
        # 绿色前缘色带与翼尖垂直安定面。
        v=box(base,f'stripe_{side}',f'{.005*sx} {side*.27*sy} .014 0 0 {side*-.43}',f'{.045*sx} {.42*sy} .003',green)
        box(base,f'winglet_{side}',f'{-.115*sx} {side*.492*sy} .045 0 0 0',f'{.13*sx} .012 .10',green)
        box(base,f'wing_collision_{side}',f'{-.04*sx} {side*.27*sy} 0 0 0 {side*-.28}',f'{.12*sx} {.43*sy} .022',white,True)
    v=sub(base,'visual',name='central_pod');sub(v,'pose',f'{.025*sx} 0 .012 0 0 0');g=sub(v,'geometry');sub(sub(g,'ellipsoid'),'radii',f'{.175*sx} {.075*sy} .04');m=sub(v,'material');sub(m,'ambient',green);sub(m,'diffuse',green)
    box(base,'canopy',f'{.07*sx} 0 .047 0 0 0',f'{.13*sx} {.07*sy} .022',black)
    box(base,'belly_collision','0 0 -.02 0 0 0',f'{.28*sx} {.1*sy} .04',white,True)
    for i,side in enumerate([1,-1]):
        link=sub(model,'link',name=f'elevon_{i}');sub(link,'pose',f'{-.14*sx} {side*.30*sy} 0 0 0 0');iner=sub(link,'inertial');sub(iner,'mass',.005);it=sub(iner,'inertia')
        for k in ['ixx','iyy','izz']:sub(it,k,.00001)
        box(link,'surface','0 0 0 0 0 0',f'{.045*sx} {.30*sy} .012',black)
        j=sub(model,'joint',name=f'servo_{i}',type='revolute');sub(j,'parent','base_link');sub(j,'child',f'elevon_{i}');a=sub(j,'axis');sub(a,'xyz','0 1 0');lim=sub(a,'limit');sub(lim,'lower',-.5);sub(lim,'upper',.5);dyn=sub(a,'dynamics');sub(dyn,'damping',.01)
        plugin=sub(model,'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController');sub(plugin,'joint_name',f'servo_{i}');sub(plugin,'sub_topic',f'servo_{i}');sub(plugin,'p_gain',.1);sub(plugin,'d_gain',.001);sub(plugin,'cmd_max',.02);sub(plugin,'cmd_min',-.02)
    rotor=sub(model,'link',name='rotor_pusher');sub(rotor,'pose',f'{-.196*sx} 0 .025 0 1.57079632679 0');iner=sub(rotor,'inertial');sub(iner,'mass',.006);it=sub(iner,'inertia')
    for k in ['ixx','iyy','izz']:sub(it,k,.00001)
    box(rotor,'two_blade_propeller','0 0 0 0 0 0',f'.014 {p["motor"]["diameter_m"]} {.008*sx}',black)
    j=sub(model,'joint',name='rotor_pusher_joint',type='revolute');sub(j,'parent','base_link');sub(j,'child','rotor_pusher');a=sub(j,'axis');sub(a,'xyz','0 0 1')
    aero=copy.deepcopy(source.find("plugin[@name='gz::sim::systems::AdvancedLiftDrag']"));model.append(aero)
    for c in aero.findall('control_surface'):aero.remove(c)
    for k,v in p['aerodynamics'].items():setv(aero,{'area_m2':'area','mac_m':'mac'}.get(k,k),v)
    setv(aero,'cp','0 0 0');setv(aero,'AR',B*B/p['aerodynamics']['area_m2']);setv(aero,'num_ctrl_surfaces',2)
    for i,sign in enumerate([-1,1]):
        c=sub(aero,'control_surface');
        for k,v in {'name':f'servo_{i}','index':i,'direction':1,'CD_ctrl':0.0001,'CY_ctrl':0,'CL_ctrl':-.005,'Cell_ctrl':sign*.003,'Cem_ctrl':.006,'Cen_ctrl':0}.items():sub(c,k,v)
    motor=copy.deepcopy(source.find("plugin[@name='gz::sim::systems::MulticopterMotorModel']"));model.append(motor)
    for k,v in {'jointName':'rotor_pusher_joint','linkName':'rotor_pusher','maxRotVelocity':p['motor']['max_rad_s'],'motorConstant':p['motor']['thrust_coefficient'],'momentConstant':.002,'rotorDragCoefficient':.00001}.items():setv(motor,k,v)
    plugin=sub(model,'plugin',filename='libHandLaunch.so',name='assets::HandLaunch');sub(plugin,'speed',p['launch']['speed_m_s']);sub(plugin,'upward',p['launch']['upward_m_s']);sub(plugin,'mass',p['mass_kg'])
    E.indent(sdf);E.ElementTree(sdf).write(out/'model.sdf',encoding='utf-8',xml_declaration=True)
    world=E.parse(ROOT/'tools/world_template.sdf');w=world.getroot().find('world');w.set('name','xiaobianfu_test');inc=w.find('include');setv(inc,'uri','model://xiaobianfu');setv(inc,'pose','0 0 .045 0 0 0');setv(w.find('physics'),'max_step_size',.002);setv(w.find("model[@name='ground_plane']/link/visual/geometry/plane"),'size','10000 10000')
    ground_collision=w.find("model[@name='ground_plane']/link/collision");setv(ground_collision,'pose','0 0 -.05 0 0 0');cg=ground_collision.find('geometry');cg.remove(cg.find('plane'));sub(sub(cg,'box'),'size','10000 10000 .1')
    ground=w.find("model[@name='ground_plane']/link/visual/material")
    setv(ground,'ambient','.16 .32 .08 1');setv(ground,'diffuse','.16 .32 .08 1')
    marker=sub(w,'model',name='XIAOBIANFU_LOCATION_MARKER');sub(marker,'static','true');lk=sub(marker,'link',name='marker');box(lk,'bright_pole','0 -2 1 0 0 0','.15 .15 2','1 .05 .7 1');box(lk,'arrow_to_aircraft','0 -1 .08 0 0 0','.12 1.6 .05','1 1 0 1')
    E.indent(world);world.write(out/'tests/world.sdf',encoding='utf-8',xml_declaration=True)
    for f in ['model.config','LICENSE-PX4','LICENSE-PX4-GZ']:shutil.copy2(ROOT/f,out/f) if out!=ROOT else None
    for f in (ROOT/'tests').glob('*'):
        if f.is_file() and f.suffix in ['.py','.sh'] and out!=ROOT:shutil.copy2(f,out/'tests'/f.name)
    for f in (ROOT/'px4').glob('*'):shutil.copy2(f,out/'px4'/f.name) if out!=ROOT else None
    airframe=out/'px4/4024_gz_xiaobianfu'
    airframe.write_text(re.sub(r'(param set-default SIM_GZ_EC_MAX1 )[^\n]+',lambda m:m.group(1)+str(p['motor']['max_rad_s']),airframe.read_text()))
    (out/'parameters.json').write_text(json.dumps(p,ensure_ascii=False,indent=2)+'\n')
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,default=ROOT/'parameters.json');ap.add_argument('--output-dir',type=Path,default=ROOT);a=ap.parse_args();generate(json.loads(a.config.read_text()),a.output_dir.resolve())
