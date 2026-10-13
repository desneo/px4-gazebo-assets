#!/usr/bin/env python3
"""保存当前配置的飞行真值结果、摘要与图表；拒绝把不同配置标记为通过。"""
import argparse,gzip,hashlib,json,xml.etree.ElementTree as E
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('report',type=Path);a=ap.parse_args();data=json.loads(a.report.read_text())
for field,path in [('config_sha256',ROOT/'parameters.json'),('model_sha256',ROOT/'model.sdf'),('world_sha256',ROOT/'tests/world.sdf'),('airframe_sha256',ROOT/'px4/4024_gz_xiaobianfu'),('validator_sha256',ROOT/'tests/flight_check.py')]:
    actual=hashlib.sha256(path.read_bytes()).hexdigest()
    if data.get(field)!=actual:raise ValueError(f'{field}与当前文件不匹配，需重新验证')
(ROOT/'tests/results').mkdir(exist_ok=True)
with gzip.open(ROOT/'tests/results/flight_result.json.gz','wt') as f:json.dump(data,f,ensure_ascii=False)
summary={k:v for k,v in data.items() if k not in ['samples','events']};appearance=json.loads((ROOT/'docs/appearance.json').read_text());model=E.parse(ROOT/'model.sdf').getroot().find('model')
visuals=[{'link':link.get('name'),'pose':link.findtext('pose','0 0 0 0 0 0'),'visuals':[E.tostring(x,encoding='unicode') for x in link.findall('visual')]} for link in model.findall('link')]
actual_visual_hash=hashlib.sha256(json.dumps(visuals,sort_keys=True).encode()).hexdigest()
actual_mesh_hash={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT/'meshes').glob('*.stl'))}
if actual_visual_hash!=appearance['visual_sha256'] or actual_mesh_hash!=appearance['mesh_sha256']:raise ValueError('外形与用户已确认的Gazebo模型不一致，需要重新人工确认')
summary['appearance_status']=appearance['status'];summary['appearance']=appearance
(ROOT/'docs/validation.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
rows=[r for r in data['samples'] if r.get('truth')];t=[r['truth']['t'] for r in rows];speed=[r['truth']['speed_m_s']*3.6 for r in rows];alt=[r['truth']['alt_m'] for r in rows]
fig,axes=plt.subplots(3,1,figsize=(11,8),layout='constrained');axes[0].plot(t,speed,lw=1,label='Gazebo truth');axes[0].axhline(100,color='green',ls='--',label='100 km/h target');axes[0].set(ylabel='Speed (km/h)');axes[0].legend()
axes[1].plot(t,alt,lw=1,label='Gazebo truth');axes[1].axhspan(48,52,color='green',alpha=.12,label='50 m +/- 2 m');axes[1].set(ylabel='Height above grass (m)');axes[1].legend()
landing=[r for r in rows if r['stage']=='landing'];axes[2].plot([r['truth']['t'] for r in landing],[r['truth']['alt_m'] for r in landing],label='Landing height');axes[2].plot([r['truth']['t'] for r in landing],[r['truth']['speed_m_s'] for r in landing],label='Landing speed (m/s)');axes[2].set(xlabel='Simulation time (s)',ylabel='m / m/s');axes[2].legend()
for ax in axes:ax.grid(alpha=.2)
if data.get('cruise'):
 c=data['cruise']
 for ax in axes[:2]:ax.axvspan(c['start_t'],c['end_t'],alpha=.10,color='green')
fig.suptitle('Xiaobianfu | autonomous launch / maximum-power level flight / belly landing | '+('PASS' if data['passed'] else 'INCOMPLETE'));fig.savefig(ROOT/'docs/flight_validation.png',dpi=150)
print(ROOT/'docs/validation.json')
