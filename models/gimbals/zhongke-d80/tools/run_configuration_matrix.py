#!/usr/bin/env python3
"""逐一启动独立配置世界，运行实际物理/光学检查并释放自身会话。"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.session import load_session,stop_session,process_matches


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix',type=Path,default=ROOT/'runs/configuration_matrix/matrix.json')
    parser.add_argument('--case',action='append')
    parser.add_argument('--baseline-session',default='validation')
    parser.add_argument('--tracking',action='store_true')
    args=parser.parse_args()
    matrix=json.loads(args.matrix.read_text())
    result={'passed':False,'cases':[]}
    output=args.matrix.parent/'runtime_results.json'
    def save():
        output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    try:
        for case in matrix['cases']:
            name=case['name']
            if args.case and name not in args.case:
                continue
            entry={'name':name,'config_hash':case['config_hash'],'passed':False}
            result['cases'].append(entry);save()
            if name=='rate_20':
                baseline=load_session(args.baseline_session)
                report=Path(baseline['session_path']).parent/'physics/result.json'
                physical=json.loads(report.read_text())
                if physical.get('result')!='passed' or physical['config_hash']!=case['config_hash']:
                    raise AssertionError('20°/s基线尚未通过或配置hash不一致')
                entry.update(passed=True,reused_physics_evidence=str(report))
                case['runtime_validation']='passed'
                continue
            session='matrix_'+name
            directory=ROOT/'runs'/session
            directory.mkdir(parents=True,exist_ok=True)
            log=(directory/'launcher.log').open('w')
            process=subprocess.Popen([sys.executable,str(ROOT/'scripts/runtime.py'),
                '--headless','--config',case['config'],'--session',session],cwd=ROOT,
                stdout=log,stderr=subprocess.STDOUT)
            try:
                deadline=time.monotonic()+15
                while time.monotonic()<deadline:
                    if process.poll() is not None:
                        raise AssertionError(f'{name}启动失败，请查看{directory/"launcher.log"}')
                    try:
                        current=load_session(session)
                        if current['state']=='running' and current['config_hash']==case['config_hash']:
                            break
                    except FileNotFoundError:
                        pass
                    time.sleep(.1)
                else:
                    raise AssertionError(f'{name}启动清单超时')
                commands=[('runtime_config',ROOT/'tools/check_runtime_config.py')]
                if name.startswith('rate_') or name=='limits':
                    commands.append(('physics',ROOT/'tools/check_physics.py'))
                if args.tracking and name.startswith('rate_'):
                    commands.append(('tracking',ROOT/'tests/check_tracking.py'))
                if name in ('resolution_1280','focal_100mm'):
                    commands.append(('optics',ROOT/'tests/check_optics.py'))
                for label,tool in commands:
                    print(f'实际改参验收：{name} / {label}',flush=True)
                    checked=subprocess.run([sys.executable,str(tool),'--session',session],
                        cwd=ROOT,capture_output=True,text=True,timeout=180)
                    (directory/(label+'_check.log')).write_text(checked.stdout+checked.stderr)
                    if checked.returncode:
                        raise AssertionError(f'{name}/{label}失败，请查看{directory/(label+"_check.log")}')
                entry.update(passed=True,session=session,evidence=str(directory))
                case['runtime_validation']='passed'
                print(f'实际改参验收通过：{name}',flush=True)
            finally:
                if process.poll() is None:
                    stop_session(session)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.terminate();process.wait(timeout=10)
                log.close()
                try:
                    remaining=load_session(session)['processes']
                    if any(process_matches(item) for item in remaining.values()):
                        raise AssertionError(f'{name}清理后仍有本会话进程')
                except FileNotFoundError:
                    pass
                save()
        result['passed']=all(entry['passed'] for entry in result['cases'])
        matrix['runtime_validation']='passed' if all(case['runtime_validation']=='passed' for case in matrix['cases']) else 'partial'
        args.matrix.write_text(json.dumps(matrix,ensure_ascii=False,indent=2)+'\n')
    except Exception as exc:
        result['error']=str(exc)
        print(f'实际改参验收失败：{exc}',file=sys.stderr,flush=True)
    save()
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
