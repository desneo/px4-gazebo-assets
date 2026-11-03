#!/usr/bin/env bash
# 仅停止本模型的地面站；仿真、网关和视频服务继续运行。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$(cd "$HERE/.." && pwd)"
DRY_RUN=0
case "${1:-}" in
  '') ;;
  --dry-run) DRY_RUN=1 ;;
  -h|--help) echo "用法：bash $0 [--dry-run]（仅停止艾晟地面站）"; exit 0 ;;
  *) echo "用法：bash $0 [--dry-run]" >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { echo '参数过多' >&2; exit 2; }

found=0
status=0
for proc in /proc/[0-9]*; do
  [[ -O "$proc" && -r "$proc/cmdline" ]] || continue
  args=()
  mapfile -d '' -t args < "$proc/cmdline" 2>/dev/null || continue
  [[ ${#args[@]} -gt 1 ]] || continue
  [[ "${args[0]##*/}" == python* ]] || continue
  cwd="$(readlink -f "$proc/cwd" 2>/dev/null)" || continue
  matched=0
  for ((index=1; index<${#args[@]}; index++)); do
    arg="${args[index]}"
    if [[ "$arg" == -c ]]; then
      break
    elif [[ "$arg" == -m ]]; then
      if [[ "${args[index+1]:-}" == ground_station.main && "$cwd" == "$MODEL_DIR" ]]; then
        matched=1
      fi
      break
    elif [[ "$arg" == -* ]]; then
      continue
    fi
    script="$arg"
    [[ "$script" == /* ]] || script="$cwd/$script"
    script="$(readlink -f -- "$script" 2>/dev/null)" || break
    [[ "$script" == "$HERE/main.py" ]] && matched=1
    break
  done
  [[ "$matched" -eq 1 ]] || continue
  found=1
  pid="${proc##*/}"
  if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "将停止艾晟地面站：PID $pid"
  elif kill -TERM "$pid" 2>/dev/null; then
    echo "已发送地面站停止信号：PID $pid"
  elif [[ -d "$proc" ]]; then
    echo "停止地面站失败：PID $pid" >&2
    status=1
  fi
done
[[ "$found" -eq 1 ]] || echo '没有运行中的艾晟地面站。'
exit "$status"
