#!/usr/bin/env python3
"""生成中科合法改参矩阵及预期快照，不启动或停止任何世界。"""
from __future__ import annotations
import argparse
import importlib.util
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET
import math

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from common.config import load_config

CASES={
    'rate_10':{'mechanics':{'max_rate_dps':'10'}},
    'rate_20':{'mechanics':{'max_rate_dps':'20'}},
    'rate_30':{'mechanics':{'max_rate_dps':'30'}},
    'mass_240g':{'model':{'mass_kg':'0.240'}},
    'limits':{'mechanics':{'yaw_min_deg':'-45','yaw_max_deg':'45','pitch_min_deg':'-60','pitch_max_deg':'20'}},
    'resolution_1280':{'optics':{'width_px':'1280','height_px':'720'}},
    'focal_100mm':{'optics':{'focal_length_mm':'100'}},
}


def rewrite_ini(source,changes):
    section='';out=[];remaining={(s,k) for s,values in changes.items() for k in values}
    for line in source.splitlines(keepends=True):
        match=re.match(r'^\s*\[([^]]+)\]\s*$',line.strip())
        if match:section=match.group(1)
        field=re.match(r'^(\s*)([A-Za-z0-9_]+)\s*=.*?(\r?\n)?$',line)
        if field and section in changes and field.group(2) in changes[section]:
            key=field.group(2);line=f'{field.group(1)}{key} = {changes[section][key]}\n';remaining.remove((section,key))
        out.append(line)
    if remaining:raise ValueError(f'原始INI缺少矩阵字段：{sorted(remaining)}')
    return ''.join(out)


def validate_generated(destination,c):
    model=ET.parse(destination/'model.sdf').getroot().find('model')
    total=sum(float(link.findtext('inertial/mass')) for link in model.findall('link'))
    if not math.isclose(total,c['model']['mass_kg'],rel_tol=1e-12,abs_tol=1e-14):raise AssertionError('生成质量与矩阵配置不符')
    plugin=model.find('plugin')
    if plugin.findtext('config_hash')!=c['config_hash']:raise AssertionError('生成hash与矩阵配置不符')
    for axis in ('yaw','pitch'):
        joint=model.find(f"joint[@name='{axis}_joint']")
        for key,config_key in [('lower',axis+'_min_deg'),('upper',axis+'_max_deg')]:
            actual=math.degrees(float(joint.findtext('axis/limit/'+key)))
            if not math.isclose(actual,c['mechanics'][config_key],abs_tol=1e-10):raise AssertionError('生成机械范围不符')
        rate=math.degrees(float(joint.findtext('axis/limit/velocity')))
        if not math.isclose(rate,c['mechanics']['max_rate_dps'],abs_tol=1e-10):raise AssertionError('生成实际速度限制不符')
    camera=model.find('.//sensor/camera')
    for key in ('fx','fy','cx','cy'):
        if not math.isclose(float(camera.findtext('lens/intrinsics/'+key)),c['derived'][key],rel_tol=1e-12):
            raise AssertionError('生成相机内参与矩阵配置不符')
    for key,field in [('width','width_px'),('height','height_px')]:
        if int(camera.findtext('image/'+key))!=c['optics'][field]:raise AssertionError('生成分辨率不符')
    return {'generated_content':'passed','total_mass_kg':total,'config_hash':c['config_hash']}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'parameters.ini')
    parser.add_argument('--output',type=Path,default=ROOT/'runs/configuration_matrix')
    parser.add_argument('--case',choices=tuple(CASES),action='append')
    parser.add_argument('--generate-models',action='store_true',help='额外生成并静态核对各矩阵SDF；仍不启动世界')
    args=parser.parse_args();base=load_config(args.config);text=args.config.read_text()
    args.output.mkdir(parents=True,exist_ok=True)
    generator=None
    if args.generate_models:
        spec=importlib.util.spec_from_file_location('zhongke_generator',ROOT/'tools/generate_model.py')
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
    manifest={'source_config':str(args.config.resolve()),'source_hash':base['config_hash'],
              'runtime_validation':'not_run','description':'合法改参副本和预期快照；实际生效须分别重启会话并运行物理/光学验收',
              'cases':[]}
    for name in args.case or CASES:
        path=args.output/(name+'.ini');path.write_text(rewrite_ini(text,CASES[name]))
        c=load_config(path)
        entry={'name':name,'config':str(path.resolve()),'config_hash':c['config_hash'],
               'changes':CASES[name],'mass_kg':c['model']['mass_kg'],'mechanics':c['mechanics'],
               'optics':c['optics'],'derived':c['derived'],'configuration_validation':'passed',
               'runtime_validation':'not_run'}
        if generator:
            destination=args.output/name/'generated';generator.generate(path,destination)
            entry['generated']=validate_generated(destination,c)
            entry['generated_path']=str(destination.resolve())
        manifest['cases'].append(entry)
    (args.output/'matrix.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(f'已准备{len(manifest["cases"])}组合法改参配置：{args.output}；实际重启/物理/渲染验收尚未运行。')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
