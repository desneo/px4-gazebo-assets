#!/usr/bin/env python3
"""A2416A 地面站进程登记与定向停止。"""
from __future__ import annotations

import argparse
import configparser
import fcntl
import hashlib
import os
from pathlib import Path
import secrets
import signal
import stat
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
MAIN_MODULE = "ground_station.main"
TOKEN_ENV = "JIELIAN_A2416A_GCS_TOKEN"
UID = os.getuid()


def runtime_dir() -> Path:
    """返回仅当前用户可写的运行清单目录。"""
    candidates = []
    if os.environ.get("XDG_RUNTIME_DIR"):
        candidates.append(Path(os.environ["XDG_RUNTIME_DIR"]) / "jielian-a2416a-gcs")
    candidates.append(Path(f"/tmp/jielian-a2416a-{UID}"))
    last_error = None
    for path in candidates:
        try:
            if path.exists():
                info = path.lstat()
                if not stat.S_ISDIR(info.st_mode) or info.st_uid != UID:
                    raise RuntimeError(f"运行目录身份不安全：{path}")
                os.chmod(path, 0o700)
            else:
                path.mkdir(mode=0o700, parents=True)
            return path
        except OSError as exc:
            last_error = exc
    raise RuntimeError(f"无法创建A2416A地面站运行目录：{last_error}")


def canonical_config(value: str | os.PathLike[str], *, require_absolute: bool = False) -> Path:
    path = Path(value).expanduser()
    if require_absolute and not path.is_absolute():
        raise ValueError("--config 必须使用绝对路径")
    if not path.is_absolute():
        path = Path.cwd() / path
    return path.resolve(strict=False)


def instance_id(config: Path) -> str:
    return hashlib.sha256(os.fsencode(str(config))).hexdigest()[:24]


def state_path(config: Path) -> Path:
    return runtime_dir() / f"{instance_id(config)}.ini"


def lock_path(config: Path) -> Path:
    return runtime_dir() / f"{instance_id(config)}.lock"


def process_start_ticks(pid: int) -> int:
    raw = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    return int(raw[raw.rfind(")") + 2:].split()[19])


def process_uid(pid: int) -> int:
    for line in Path(f"/proc/{pid}/status").read_text(encoding="ascii").splitlines():
        if line.startswith("Uid:"):
            return int(line.split()[1])
    raise ValueError("进程状态缺少Uid")


def _proc_arguments(pid: int) -> list[str]:
    return [os.fsdecode(item) for item in
            Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0") if item]


def _proc_environment(pid: int) -> dict[str, str]:
    result = {}
    for item in Path(f"/proc/{pid}/environ").read_bytes().split(b"\0"):
        if b"=" in item:
            name, value = item.split(b"=", 1)
            result[os.fsdecode(name)] = os.fsdecode(value)
    return result


def _module_arguments(command: list[str]) -> list[str] | None:
    """只接受 python [安全解释器选项] -m ground_station.main。"""
    index = 1
    safe_flags = {"-B", "-E", "-I", "-O", "-OO", "-P", "-q", "-s", "-S", "-u"}
    while index < len(command) and command[index] in safe_flags:
        index += 1
    if command[index:index + 2] != ["-m", MAIN_MODULE]:
        return None
    return command[index + 2:]


def _argument_value(arguments: list[str], name: str) -> str | None:
    for index, value in enumerate(arguments):
        if value == name and index + 1 < len(arguments):
            return arguments[index + 1]
        if value.startswith(name + "="):
            return value[len(name) + 1:]
    return None


def _arguments_config(arguments: list[str]) -> Path:
    value = _argument_value(arguments, "--config")
    if value is None:
        return canonical_config(ROOT / "ground_station/config.ini")
    path = Path(value).expanduser()
    return canonical_config(path if path.is_absolute() else ROOT / path)


def read_state(path: Path) -> dict[str, str] | None:
    try:
        info = path.lstat()
        if not stat.S_ISREG(info.st_mode) or info.st_uid != UID or info.st_mode & 0o022:
            return None
        parser = configparser.ConfigParser(interpolation=None)
        with path.open("r", encoding="utf-8") as stream:
            parser.read_file(stream)
        return dict(parser["process"])
    except (OSError, KeyError, configparser.Error):
        return None


def write_state(path: Path, values: dict[str, object]) -> None:
    parser = configparser.ConfigParser(interpolation=None)
    parser["process"] = {name: str(value) for name, value in values.items()}
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            parser.write(stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _identity_matches(record: dict[str, str], *, token: str | None) -> bool:
    """同时核对PID代际、用户、解释器、模块、配置与工作目录。"""
    try:
        pid = int(record["pid"])
        if process_uid(pid) != UID or process_start_ticks(pid) != int(record["start_ticks"]):
            return False
        command = _proc_arguments(pid)
        if not command or Path(f"/proc/{pid}/exe").resolve() != Path(record["executable"]).resolve():
            return False
        arguments = _module_arguments(command)
        if arguments is None:
            return False
        if _arguments_config(arguments) != Path(record["config"]):
            return False
        if Path(f"/proc/{pid}/cwd").resolve() != ROOT.resolve():
            return False
        if token is not None and _proc_environment(pid).get(TOKEN_ENV) != token:
            return False
        return True
    except (OSError, ValueError, KeyError, IndexError):
        return False


def is_managed_process(record: dict[str, str]) -> bool:
    """登记进程还须匹配启动时写入的随机身份。"""
    try:
        return _identity_matches(record, token=record["token"])
    except KeyError:
        return False


def _open_lock(config: Path):
    descriptor = os.open(lock_path(config), os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    stream = os.fdopen(descriptor, "w")
    try:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        os.set_inheritable(stream.fileno(), True)
        return stream
    except Exception:
        stream.close()
        raise


def _normalise_run_arguments(arguments: list[str]) -> tuple[list[str], Path]:
    value = _argument_value(arguments, "--config")
    config = canonical_config(value or ROOT / "ground_station/config.ini")
    result: list[str] = []
    skip = False
    for item in arguments:
        if skip:
            skip = False
            continue
        if item == "--config":
            skip = True
            continue
        if item.startswith("--config="):
            continue
        result.append(item)
    return ["--config", str(config), *result], config


def run(arguments: list[str]) -> int:
    command_arguments, config = _normalise_run_arguments(arguments)
    try:
        lock = _open_lock(config)
    except BlockingIOError:
        print(f"该配置的A2416A地面站已运行：{config}", file=sys.stderr)
        return 2
    legacy = discover_unregistered_processes(config)
    if legacy:
        lock.close()
        pids = ", ".join(record["pid"] for record in legacy)
        print(f"该配置已有修改前或直接启动的A2416A地面站：{config}（PID {pids}）",
              file=sys.stderr)
        return 2
    token = secrets.token_hex(24)
    path = state_path(config)
    write_state(path, {
        "pid": os.getpid(),
        "start_ticks": process_start_ticks(os.getpid()),
        "uid": UID,
        "executable": Path(sys.executable).resolve(),
        "module": MAIN_MODULE,
        "root": ROOT.resolve(),
        "config": config,
        "token": token,
        "started_unix_ns": time.time_ns(),
    })
    os.environ[TOKEN_ENV] = token
    os.chdir(ROOT)
    os.set_inheritable(lock.fileno(), True)
    os.execv(sys.executable, [sys.executable, "-m", MAIN_MODULE, *command_arguments])
    return 1


def discover_states(config: Path | None = None) -> list[tuple[Path, dict[str, str]]]:
    if config is not None:
        candidates = [state_path(config)]
    else:
        candidates = sorted(runtime_dir().glob("*.ini"))
    result = []
    for path in candidates:
        record = read_state(path)
        if record is None:
            continue
        if config is not None and record.get("config") != str(config):
            continue
        result.append((path, record))
    return result


def discover_unregistered_processes(config: Path | None = None) -> list[dict[str, str]]:
    """兼容升级前已启动实例；仍严格核对用户、PID代际、命令和程序目录。"""
    result = []
    for process in sorted(Path("/proc").glob("[0-9]*"), key=lambda item: int(item.name)):
        try:
            pid = int(process.name)
            if process_uid(pid) != UID:
                continue
            command = _proc_arguments(pid)
            arguments = _module_arguments(command)
            if arguments is None or Path(f"/proc/{pid}/cwd").resolve() != ROOT.resolve():
                continue
            process_config = _arguments_config(arguments)
            if config is not None and process_config != config:
                continue
            record = {
                "pid": str(pid),
                "start_ticks": str(process_start_ticks(pid)),
                "executable": str(Path(f"/proc/{pid}/exe").resolve()),
                "config": str(process_config),
            }
            # 采集后再完整核对一次，避免PID恰好在扫描期间被复用。
            if _identity_matches(record, token=None):
                result.append(record)
        except (OSError, ValueError, IndexError):
            continue
    return result


def signal_verified_process(record: dict[str, str], sig: int) -> bool:
    """通过pidfd固定进程代际，复核完整身份后再发送信号。"""
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise RuntimeError("当前系统不支持pidfd，拒绝执行可能误停其他进程的操作")
    pid = int(record["pid"])
    try:
        descriptor = os.pidfd_open(pid)
    except ProcessLookupError:
        return False
    try:
        if not _identity_matches(record, token=record.get("token")):
            return False
        signal.pidfd_send_signal(descriptor, sig)
        return True
    except ProcessLookupError:
        return False
    finally:
        os.close(descriptor)


def stop(config: Path | None, *, dry_run: bool) -> int:
    matched = 0
    status = 0
    records = []
    registered_pids = set()
    for _path, record in discover_states(config):
        if not is_managed_process(record):
            continue
        records.append(record)
        registered_pids.add(record["pid"])
    records.extend(record for record in discover_unregistered_processes(config)
                   if record["pid"] not in registered_pids)
    for record in records:
        matched += 1
        pid = int(record["pid"])
        if dry_run:
            print(f"将停止A2416A地面站：PID {pid}，配置 {record['config']}")
            continue
        try:
            if signal_verified_process(record, signal.SIGTERM):
                print(f"已发送A2416A地面站停止信号：PID {pid}，配置 {record['config']}")
        except RuntimeError as exc:
            print(f"停止A2416A地面站失败：{exc}", file=sys.stderr)
            return 2
        except (OSError, ValueError) as exc:
            print(f"停止A2416A地面站失败：PID {pid}：{exc}", file=sys.stderr)
            status = 2
            continue
    if not matched:
        target = f"（配置 {config}）" if config else ""
        print(f"当前没有运行中的A2416A地面站{target}")
    return status


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if not arguments or arguments[0] not in {"run", "stop"}:
        print("进程入口需要 run 或 stop 动作", file=sys.stderr)
        return 2
    action = arguments.pop(0)
    if action == "run":
        return run(arguments)
    parser = argparse.ArgumentParser(description="仅停止A2416A地面站进程")
    parser.add_argument("--config", help="只停止使用该绝对配置路径的实例")
    parser.add_argument("--dry-run", action="store_true", help="仅预览，不发送信号")
    options = parser.parse_args(arguments)
    try:
        config = canonical_config(options.config, require_absolute=True) if options.config else None
        return stop(config, dry_run=options.dry_run)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"A2416A地面站停止失败：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
