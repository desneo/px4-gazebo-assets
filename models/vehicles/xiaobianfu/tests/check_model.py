#!/usr/bin/env python3
"""检查整机名义参数、质量守恒、系统与物理手抛入口。"""
from pathlib import Path
import json, xml.etree.ElementTree as E
R=Path(__file__).resolve().parents[1];p=json.loads((R/'parameters.json').read_text());m=E.parse(R/'model.sdf').getroot().find('model')
assert p['dimensions']=={'length_m':.4,'span_m':1.0}
assert abs(sum(float(x.text) for x in m.findall('link/inertial/mass'))-1)<1e-9
assert {s.get('type') for s in m.findall('link/sensor')} >= {'imu','magnetometer','navsat','air_pressure'}
assert len(m.findall("plugin[@name='gz::sim::systems::AdvancedLiftDrag']/control_surface"))==2
assert len(m.findall("joint[@type='revolute']"))==3
for mesh in m.findall('.//mesh/uri'):assert (R/mesh.text.split('model://xiaobianfu/')[1]).exists()
assert 'RWTO_TKOFF 0' in (R/'px4/4024_gz_xiaobianfu').read_text()
print('PASS: nominal dimensions, 1kg mass, sensors, elevons, pusher and hand-launch configuration')
# 由实际SDF视觉几何和STL建立包围盒，包含link/visual旋转和尾桨厚度。
import math,itertools,numpy as np
def transform(pose):
    v=list(map(float,(pose or '0 0 0 0 0 0').split()));x,y,z,r,p,yaw=v
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(yaw),math.sin(yaw)
    rot=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    return rot,np.array([x,y,z])
points=[]
for link in m.findall('link'):
    lr,lt=transform(link.findtext('pose'))
    for vis in link.findall('visual'):
        vr,vt=transform(vis.findtext('pose'));rot=lr@vr;trans=lr@vt+lt;g=vis.find('geometry')
        if g.find('mesh') is not None:
            path=R/g.findtext('mesh/uri').split('model://xiaobianfu/')[1]
            for line in path.read_text().splitlines():
                if line.startswith('vertex'):points.append(rot@np.array(list(map(float,line.split()[1:])))+trans)
        elif g.find('box') is not None:
            half=np.array(list(map(float,g.findtext('box/size').split())))/2
            for sign in itertools.product([-1,1],repeat=3):points.append(rot@(half*np.array(sign))+trans)
        elif g.find('ellipsoid') is not None:
            radii=np.array(list(map(float,g.findtext('ellipsoid/radii').split())))
            extent=np.sqrt(np.sum((rot*radii)**2,axis=1));points.extend([trans-extent,trans+extent])
bounds=np.array(points);size=bounds.max(axis=0)-bounds.min(axis=0)
assert abs(size[0]-.4)<1e-9, size
assert abs(size[1]-1)<1e-9, size
print('PASS: actual visual bounding box length=%.3fm span=%.3fm'%(size[0],size[1]))
w=E.parse(R/'tests/world.sdf').getroot().find('world')
ground=w.find("model[@name='ground_plane']/link/collision")
assert ground.find('geometry/box') is not None, '远距离着陆须使用明确覆盖范围的碰撞箱体'
assert min(map(float,ground.findtext('geometry/box/size').split()[:2]))>=10000
assert abs(float(w.findtext("gui/plugin[@filename='MinimalScene']/camera_clip/near"))-.005)<1e-9
assert w.find("model[@name='XIAOBIANFU_LOCATION_MARKER']//collision") is None
print('PASS: 10km grass collision surface, close camera and collision-free marker')
