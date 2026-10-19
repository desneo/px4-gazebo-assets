#!/usr/bin/env python3
"""Linux Station 与本机指定 bridge 的启动、依赖恢复和停止。"""
from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time

import ground_station as common


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SCRIPTS = HERE.parent / "scripts"
BRIDGE = HERE.parent / "controller/service/gimbal_bridge.py"


def option(argv, name, default):
    value = default
    for index, arg in enumerate(argv):
        if arg == name and index + 1 < len(argv):
            value = argv[index + 1]
        elif arg.startswith(name + "="):
            value = arg.split("=", 1)[1]
    return value


def process_info(pid: int):
    """同 UID 的进程快照；starttime 防止 PID 复用导致误停。"""
    proc = Path(f"/proc/{pid}")
    try:
        if proc.stat().st_uid != os.getuid():
            return None
        stat = (proc / "stat").read_text().rsplit(")", 1)[1].split()
        if stat[0] == "Z":
            return None
        argv = (proc / "cmdline").read_bytes().decode().strip("\0").split("\0")
        return {"pid": pid, "starttime": stat[19], "ppid": int(stat[1]),
                "argv": argv}
    except (OSError, ValueError, UnicodeError):
        return None


def same_process(record):
    if not record:
        return False
    live = process_info(record["pid"])
    return bool(live and live["starttime"] == record["starttime"]
                and live["argv"] == record["argv"])


def matches_bridge(argv, config):
    return (str(BRIDGE) in argv
            and option(argv, "--udp-port", "8766") == str(config["udp_port"])
            and option(argv, "--aircraft-id", "1") == str(config["aircraft_id"]))


def local_host(host):
    for family, _, _, _, addr in socket.getaddrinfo(host, 0, type=socket.SOCK_DGRAM):
        try:
            with socket.socket(family, socket.SOCK_DGRAM) as probe:
                probe.bind(addr)
            return True
        except OSError:
            continue
    return False


def stop_process(record, timeout=8):
    if not same_process(record):
        return
    os.kill(record["pid"], signal.SIGINT)
    started = time.monotonic()
    deadline = started + timeout
    terminated = False
    while same_process(record) and time.monotonic() < deadline:
        if not terminated and time.monotonic() - started > 2:
            # 后台旧网关可能继承 SIGINT=ignore，改发标准 SIGTERM。
            os.kill(record["pid"], signal.SIGTERM)
            terminated = True
        time.sleep(.1)
    if same_process(record):
        raise RuntimeError(f"进程 {record['pid']} 未正常停止，请检查日志；没有强制终止。")


def wait_rtsp_release(config):
    sys.path.insert(0, str(BRIDGE.parent))
    from rtsp_relay import request

    mount = "/" + config["rtsp_mount"].strip("/") + f"_{config['aircraft_id']}"
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        try:
            if mount not in request(config["rtsp_port"], {"op": "ping"}).get("paths", []):
                return
        except (OSError, RuntimeError, ValueError):
            return
        time.sleep(.25)
    raise RuntimeError(f"旧 RTSP 路径 {mount} 尚未释放，拒绝覆盖已有视频。")


def write_state(path, state):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)


def wait_ready(config, record):
    deadline = time.monotonic() + 12
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(.3)
        target = (socket.gethostbyname(config["gimbal_host"]), config["udp_port"])
        try:
            while same_process(record) and time.monotonic() < deadline:
                sock.sendto(b'{"cmd":"ping"}', target)
                try:
                    data, peer = sock.recvfrom(8192)
                    msg = json.loads(data)
                except (socket.timeout, ValueError):
                    continue
                if peer != target or msg.get("type") != "hello":
                    continue
                if msg.get("aircraft_id") != config["aircraft_id"]:
                    raise RuntimeError("网关 hello 的飞机 ID 与配置不一致。")
                return
        finally:
            sock.sendto(b'{"cmd":"bye"}', target)
    raise RuntimeError("网关未就绪，请查看本次 bridge.log。")


def ensure_bridge(config, state_path):
    if not local_host(config["gimbal_host"]):
        raise RuntimeError("[bridge] manage=true 只管理本机网关；远端地址请设 manage=false，"
                           "并在网关主机运行 run_bridge.sh 初始化依赖。")
    records = []
    for path in Path("/proc").iterdir():
        if path.name.isdecimal():
            record = process_info(int(path.name))
            if record and matches_bridge(record["argv"], config):
                records.append(record)
    if len(records) > 1:
        raise RuntimeError("发现多个相同 ID/端口的网关，拒绝自动管理。")
    record = records[0] if records else None
    environment = dict(os.environ)
    cwd = str(ROOT)
    pid_paths = []
    if record:
        if (option(record["argv"], "--rtsp-port", "8555") != str(config["rtsp_port"])
                or option(record["argv"], "--mount", "/two_axis_pod") != config["rtsp_mount"]):
            raise RuntimeError("现有网关 RTSP 端口/路径与 Station 配置不一致，拒绝接管。")
        proc = Path(f"/proc/{record['pid']}")
        environment = dict(item.split("=", 1) for item in
                           (proc / "environ").read_bytes().decode().strip("\0").split("\0")
                           if "=" in item)
        cwd = str((proc / "cwd").resolve())
        argv = record["argv"]
        python = argv[0]
        if not os.path.isabs(python):
            python = shutil.which(python, path=environment.get("PATH")) or python
        vision_config = option(argv, "--vision-config", str(BRIDGE.with_name("vision.json")))
        if environment.get("GIMBAL_RUNDIR"):
            pid_paths.append(Path(environment["GIMBAL_RUNDIR"]) / "bridge.pid")
        log_path = (proc / "fd/1").resolve()
        pid_paths.append(log_path.parent.parent / "pids" / (log_path.stem + ".pid"))
        parent = process_info(record["ppid"])
        if parent and not (Path(parent["argv"][0]).name == "systemd"
                           and "--user" in parent["argv"]):
            raise RuntimeError("现有网关由其他启动器监管，请先通过其 stop 脚本停止，"
                               "或设 [bridge] manage=false 仅连接。")
        check = subprocess.run([python, str(SCRIPTS / "check_bridge_env.py"),
                                "--vision-config", vision_config], cwd=cwd, env=environment)
        if check.returncode == 0:
            wait_ready(config, record)
            print(f"[station] 接管本机 {config['aircraft_id']} 号机 bridge pid={record['pid']}", flush=True)
            return record
        print(f"[station] 网关 pid={record['pid']} 的 Python 环境不完整，准备恢复。", flush=True)
    else:
        argv = [sys.executable, "-u", str(BRIDGE),
                "--world", config["bridge_world"], "--gimbal-model", config["bridge_model"],
                "--topic-prefix", config["bridge_topic_prefix"],
                "--udp-port", str(config["udp_port"]), "--rtsp-port", str(config["rtsp_port"]),
                "--aircraft-id", str(config["aircraft_id"]), "--mount", config["rtsp_mount"],
                "--encoder", "nvenc"]
        if config["bridge_image_topic"]:
            argv += ["--image-topic", config["bridge_image_topic"]]
        if config["bridge_gz_partition"]:
            environment["GZ_PARTITION"] = config["bridge_gz_partition"]
        vision_config = str(BRIDGE.with_name("vision.json"))

    # 先准备新环境，成功后再停旧进程。
    prepare_env = dict(environment)
    prepare_env.pop("GIMBAL_PYTHON", None)
    prepare_env.pop("PYTHON", None)
    subprocess.run(["bash", str(SCRIPTS / "run_bridge.sh"), "--prepare-only",
                    "--vision-config", vision_config], cwd=cwd, env=prepare_env, check=True)
    old_record = record
    if record:
        stop_process(record)
        wait_rtsp_release(config)
    python = str(ROOT / ".venv-linux/bin/python")
    environment["GIMBAL_PYTHON"] = python
    command = [python] + argv[1:]
    with (state_path.parent / "bridge.log").open("a") as log:
        child = subprocess.Popen(command, cwd=cwd, env=environment,
                                 stdin=subprocess.DEVNULL, stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True)
    record = process_info(child.pid)
    if record is None:
        raise RuntimeError("网关启动失败，请查看 bridge.log。")
    try:
        wait_ready(config, record)
    except Exception:
        stop_process(record)
        raise
    for path in pid_paths:
        if path.is_file() and path.read_text().strip() == str(old_record["pid"]):
            path.write_text(str(record["pid"]) + "\n")
    print(f"[station] bridge 已启动 pid={record['pid']}；日志 {state_path.parent / 'bridge.log'}", flush=True)
    return record


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    action = argv.pop(0)
    # 帮助与诊断不接管或改变网关。
    if action == "run" and any(arg.split("=", 1)[0] in
                               {"-h", "--help", "--selftest", "--decode-test"} for arg in argv):
        return subprocess.call([sys.executable, str(HERE / "ground_station.py"), *argv])
    parser = common.build_parser()
    args = parser.parse_args(argv)
    config = common.apply_cli_overrides(common.load_config(args.config), args)
    if args.ws_port is not None and args.udp_port is None:
        config["udp_port"] = args.ws_port
    directory = ROOT / "runs" / f"station_{config['aircraft_id']}_{config['udp_port']}"
    directory.mkdir(parents=True, exist_ok=True)
    state_path = directory / "session.json"
    state = json.loads(state_path.read_text()) if state_path.exists() else {}
    if action == "stop":
        supervisor = state.get("supervisor")
        if same_process(supervisor):
            stop_process(supervisor, timeout=20)
        elif same_process(state.get("bridge")):
            stop_process(state["bridge"])
        print("[station] Station 与本次管理的 bridge 已停止。", flush=True)
        return 0
    with (directory / "session.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("该飞机的 Station 已运行，请先执行 stop.sh。") from exc
        bridge = ensure_bridge(config, state_path) if config["bridge_manage"] else None
        gui = None
        try:
            state = {"supervisor": process_info(os.getpid()), "bridge": bridge}
            write_state(state_path, state)

            def stop_handler(*_):
                raise KeyboardInterrupt

            signal.signal(signal.SIGINT, stop_handler)
            signal.signal(signal.SIGTERM, stop_handler)
            gui = subprocess.Popen([sys.executable, "-u", str(HERE / "ground_station.py"), *argv])
            return gui.wait()
        except KeyboardInterrupt:
            return 0
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            try:
                if gui is not None and gui.poll() is None:
                    gui.terminate()
                    gui.wait(timeout=8)
            finally:
                if bridge:
                    stop_process(bridge)
                state_path.unlink(missing_ok=True)
            print("[station] Station 与对应 bridge 已关闭。", flush=True)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as exc:
        raise SystemExit(f"[station] {exc}") from exc
