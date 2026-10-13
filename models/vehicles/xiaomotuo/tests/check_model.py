#!/usr/bin/env python3
"""检查固定翼质量、尺寸、通用尾部碰撞与PX4原生控制接口。"""
import json,math,subprocess,tempfile
from pathlib import Path
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
def check(folder,config):
    model=E.parse(folder/'model.sdf').getroot().find('model')
    mass=sum(float(l.findtext('inertial/mass','0')) for l in model.findall('link'))
    assert math.isclose(mass,config['mass_kg'],abs_tol=1e-8),(mass,config['mass_kg'])
    for name in ('LeftWheelJoint','RightWheelJoint','CenterWheelJoint'):
        assert float(model.find(f"joint[@name='{name}']/axis/dynamics/friction").text)==config['wheel_axle_friction_nm']
    assert len(model.findall("plugin[@name='gz::sim::systems::AdvancedLiftDrag']"))==1
    for n in ('servo_0','servo_1','servo_2','servo_3','LeftWheelJoint','RightWheelJoint','CenterWheelJoint'):assert model.find(f"joint[@name='{n}']") is not None,n
    base=model.find("link[@name='base_link']")
    assert len([c for c in base.findall('collision') if 'boom_collision' in c.get('name')])==2
    assert base.find("collision[@name='tailplane_collision']") is not None
    vertices=[]
    ns={'d':'http://www.collada.org/2005/11/COLLADASchema'}
    for visual in model.findall('.//visual'):
        geometry=visual.find('geometry/mesh')
        if geometry is None:continue
        if visual.get('name')=='decorative_propeller':continue
        path=folder/'meshes'/Path(geometry.findtext('uri')).name
        dae=E.parse(path);source=next(s for s in dae.findall('.//d:source',ns) if s.get('id','').endswith('-positions'))
        data=list(map(float,source.find('d:float_array',ns).text.split()));scale=list(map(float,geometry.findtext('scale','1 1 1').split()))
        vertices.extend([[data[i+j]*scale[j] for j in range(3)] for i in range(0,len(data),3)])
    assert math.isclose(max(v[1] for v in vertices)-min(v[1] for v in vertices),config['span_m'],abs_tol=1e-6)
    assert math.isclose(max(v[0] for v in vertices)-min(v[0] for v in vertices),config['length_m'],abs_tol=1e-6)
    motor=model.find("plugin[@name='gz::sim::systems::MulticopterMotorModel']")
    assert float(motor.findtext('motorConstant'))==config['motor_constant']
    assert float(motor.findtext('maxRotVelocity'))==config['motor_max_rad_s']
    airframe=(folder/'px4/4030_gz_xiaomotuo').read_text()
    import re
    assert float(re.search(r'SIM_GZ_EC_MAX1 ([^\n]+)',airframe).group(1))==config['motor_max_rad_s']
config=json.loads((ROOT/'parameters.json').read_text());check(ROOT,config)
with tempfile.TemporaryDirectory(prefix='xiaomotuo-check-') as tmp:
    out=Path(tmp);cfg=dict(config,length_m=3.63,span_m=2.75,motor_max_rad_s=1200);(out/'custom.json').write_text(json.dumps(cfg))
    subprocess.run(['python3',str(ROOT/'tools/generate_model.py'),'--config',str(out/'custom.json'),'--output-dir',str(out/'model')],check=True);check(out/'model',cfg)
print('质量/尺寸缩放/碰撞/控制接口检查通过')
