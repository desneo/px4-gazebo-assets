#!/usr/bin/env python3
"""顺序运行当前单机、组合体，保留两个独立试飞的原始结果。"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument('--config', type=Path, default=HERE / 'config.json')
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    directory = (args.run_dir or ROOT / 'runs/rocket_speed_comparison' /
                 datetime.now().strftime('%Y%m%dT%H%M%S')).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    if any(directory.iterdir()):
        parser.error('对照运行目录必须为空，避免把既有结果当成本次证据')
    # 两次试飞读取同一份冻结的运行配置。
    config = directory / 'comparison_config.json'
    config.write_bytes(args.config.read_bytes())
    launcher_path = directory / 'launcher_results.json'
    launchers = {variant: {'returncode': None, 'status': 'not_started'}
                 for variant in ('standalone', 'assembled')}
    def save_launchers():
        temporary = launcher_path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(launchers, indent=2, ensure_ascii=False)+'\n')
        temporary.replace(launcher_path)
    def interrupt(signum, frame):
        raise KeyboardInterrupt(f'信号 {signum}')
    signal.signal(signal.SIGTERM, interrupt)
    save_launchers()
    failed = False
    interrupted = False
    for variant in ('standalone', 'assembled'):
        command = ['bash', str(HERE / 'run.sh'), '--variant', variant,
                   '--run-dir', str(directory / variant), '--config', str(config)]
        if args.gui:
            command.append('--gui')
        print(f'开始 {variant}：{directory / variant}', flush=True)
        launchers[variant] = {'returncode': None, 'status': 'running',
                             'started_at_unix_s': time.time(), 'command': command}
        save_launchers()
        process = None
        try:
            process = subprocess.Popen(command, start_new_session=True)
            returncode = process.wait()
        except KeyboardInterrupt:
            interrupted = True
            if process is not None:
                # 停止入口按归属请求降落；若启动器尚未记录 PID，直接发 TERM。
                pid_file = directory / variant / 'launcher.pid'
                if pid_file.is_file():
                    subprocess.run(['bash', str(HERE / 'stop.sh'), str(directory / variant)], check=False)
                elif process.poll() is None:
                    process.terminate()
                process.wait()
            returncode = 130
        except OSError as error:
            launchers[variant]['error'] = str(error)
            returncode = 127
        launchers[variant].update(returncode=returncode, status='completed',
                                 finished_at_unix_s=time.time())
        save_launchers()
        if returncode:
            failed = True
            print(f'{variant} 未完成正常试飞，保留失败证据。', flush=True)
            # 被人工停止时不继续启动第二架飞机。
            if returncode in (130, 143, -2, -15):
                interrupted = True
                break
    command = [sys.executable, str(HERE / 'report_comparison.py'),
               '--standalone', str(directory / 'standalone'),
               '--assembled', str(directory / 'assembled'),
               '--output', str(directory / 'comparison.json'),
               '--launcher-results', str(launcher_path)]
    result = subprocess.run(command)
    return 130 if interrupted else (1 if failed else result.returncode)


if __name__ == '__main__':
    raise SystemExit(main())
