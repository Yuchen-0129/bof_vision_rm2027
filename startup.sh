#!/usr/bin/env bash
# Ubuntu 24「启动应用程序」自启：cd 项目目录 → ./build/auto_aim_debug_aimer，退出后自动重启
#
# 设置 → 应用程序 → 启动 → 添加
#   命令：本脚本绝对路径
#   chmod +x startup.sh

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIDFILE="${SCRIPT_DIR}/logs/bof_vision.pid"
EXEC="${SCRIPT_DIR}/build/auto_aim_debug_aimer"
CONFIG="${SCRIPT_DIR}/configs/standard4.yaml"

mkdir -p "${SCRIPT_DIR}/logs"
LOGFILE="${SCRIPT_DIR}/logs/$(date +%Y-%m-%d_%H-%M-%S).log"

# 旧实例还在跑就直接退出
if [[ -f "${PIDFILE}" ]]; then
  pid="$(cat "${PIDFILE}")"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    echo "already running pid=${pid}" >> "${LOGFILE}"
    exit 0
  fi
  rm -f "${PIDFILE}"
fi

# 可执行不存在直接报错（避免 while true 疯狂循环）
if [[ ! -x "${EXEC}" ]]; then
  echo "[$(date -Iseconds)] FATAL: ${EXEC} not found or not executable" >> "${LOGFILE}"
  exit 1
fi

# 等串口设备就绪：避免 cboard 还没 enumerate 程序就启动失败疯狂重启
# 从 yaml 读 com_port；超时 60s 仍没就放弃（让 while true 重启走兜底）
SERIAL_PORT="$(grep -E '^com_port:' "${CONFIG}" | head -1 | sed -E 's/.*"([^"]+)".*/\1/')"

nohup bash -c "
  cd '${SCRIPT_DIR}'
  sleep 5
  if [[ -n '${SERIAL_PORT}' ]]; then
    echo \"[\$(date -Iseconds)] waiting for ${SERIAL_PORT}\"
    for i in {1..60}; do
      [[ -e '${SERIAL_PORT}' ]] && break
      sleep 1
    done
    if [[ ! -e '${SERIAL_PORT}' ]]; then
      echo \"[\$(date -Iseconds)] WARN: ${SERIAL_PORT} not ready after 60s, continuing anyway\"
    else
      echo \"[\$(date -Iseconds)] ${SERIAL_PORT} ready\"
    fi
  fi
  while true; do
    echo \"[\$(date -Iseconds)] starting ${EXEC}\"
    '${EXEC}' '${CONFIG}' || true
    echo \"[\$(date -Iseconds)] exited, restart in 1s\"
    sleep 1
  done
" >> "${LOGFILE}" 2>&1 &

echo $! > "${PIDFILE}"
echo "[$(date -Iseconds)] launched pid=$!" >> "${LOGFILE}"
