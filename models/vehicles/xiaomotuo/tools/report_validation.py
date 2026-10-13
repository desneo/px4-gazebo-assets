#!/usr/bin/env python3
"""压缩原始验收记录并生成当前配置的摘要和飞行曲线。"""
import argparse,gzip,hashlib,json,os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/xiaomotuo-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('--flight',type=Path,default=ROOT/'tests/flight_result.json');a=ap.parse_args()
def read(path):return json.loads(gzip.open(path,'rt').read() if path.suffix=='.gz' else path.read_text())
report=read(a.flight);results=ROOT/'tests/results';results.mkdir(exist_ok=True)
report['passed']=all((report.get('takeoff_passed'),report.get('cruise',{}).get('passed'),report.get('landing_passed'),report.get('touchdown',{}).get('passed'),report.get('stopped_seconds',0)>=3))
report['startup_airframe_matches_current']=report.get('startup_airframe_sha256')==hashlib.sha256((ROOT/'px4/4030_gz_xiaomotuo').read_bytes()).hexdigest()
report['validator_matches_current']=report.get('validator_sha256')==hashlib.sha256((ROOT/'tests/flight_check.py').read_bytes()).hexdigest()
report['test_settings_match_current']=report.get('test_settings_sha256')==hashlib.sha256((ROOT/'tests/settings.json').read_bytes()).hexdigest()
visual=json.loads((ROOT/'docs/visual_acceptance.json').read_text());report['visual_accepted_matches_current']=visual['status']=='accepted' and visual['model_sdf_sha256']==hashlib.sha256((ROOT/'model.sdf').read_bytes()).hexdigest() and all(hashlib.sha256((ROOT/'meshes'/name).read_bytes()).hexdigest()==digest for name,digest in visual['meshes'].items())
report['current_config_matches']=report['config_sha256']==hashlib.sha256((ROOT/'parameters.json').read_bytes()).hexdigest() and report['model_sha256']==hashlib.sha256((ROOT/'model.sdf').read_bytes()).hexdigest()
report['passed']=report['passed'] and report['current_config_matches'] and report['startup_airframe_matches_current'] and report['validator_matches_current'] and report['test_settings_match_current'] and report['visual_accepted_matches_current']
with gzip.open(results/'flight_result.json.gz','wt') as f:json.dump(report,f,separators=(',',':'))
summary={k:v for k,v in report.items() if k not in ('samples','events')};(ROOT/'docs/validation.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
rows=[r for r in report['samples'] if r.get('truth')];t0=rows[0]['truth']['t'];times=[r['truth']['t']-t0 for r in rows]
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
axes[0].plot(times,[r['truth']['speed_m_s']*3.6 for r in rows],label='Gazebo true speed');axes[0].plot(times,[r['airspeed_m_s']*3.6 for r in rows],alpha=.6,label='PX4 airspeed');axes[0].axhline(200,color='red',ls='--',label='200 km/h');axes[0].set_ylabel('Speed (km/h)');axes[0].legend()
axes[1].plot(times,[r['truth']['alt_m'] for r in rows],label='Gazebo physical height');axes[1].plot(times,[r['alt_m'] for r in rows],alpha=.6,label='PX4 estimated height');axes[1].set_ylabel('Height (m)');axes[1].legend()
axes[2].plot(times,[r['throttle_pct'] for r in rows]);axes[2].set_ylabel('Throttle (%)');axes[2].set_xlabel('Simulation seconds')
cruise=report.get('cruise',{})
if cruise.get('passed'):
    for ax in axes:ax.axvspan(cruise['start_t']-t0,cruise['end_t']-t0,color='green',alpha=.12)
for ax in axes:ax.grid(alpha=.25)
fig.suptitle('Xiaomotuo actual PX4 + Gazebo flight | '+('PASS' if report['passed'] else 'INCOMPLETE'));fig.tight_layout();fig.savefig(ROOT/'docs/flight_validation.png',dpi=150)
print(json.dumps(summary,indent=2,ensure_ascii=False))
