#!/usr/bin/env bash
# Ubuntu 24「启动应用程序」自启：cd 项目目录 → ./build/standard，退出后自动重启
#
# 设置 → 应用程序 → 启动 → 添加
#   命令：本脚本绝对路径
#   chmod +x startup.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIDFILE="${SCRIPT_DIR}/logs/bof_vision.pid"

if [[ -f "${PIDFILE}" ]]; then
  pid="$(cat "${PIDFILE}")"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    exit 0
  fi
fi

mkdir -p "${SCRIPT_DIR}/logs"
LOGFILE="${SCRIPT_DIR}/logs/$(date +%Y-%m-%d_%H-%M-%S).log"

nohup bash -c "
  cd '${SCRIPT_DIR}'
  sleep 5
  while true; do
    echo \"[\$(date -Iseconds)] starting ./build/standard\"
    ./build/standard || true
    echo \"[\$(date -Iseconds)] exited, restart in 1s\"
    sleep 1
  done
" >> "${LOGFILE}" 2>&1 &

echo $! > "${PIDFILE}"
