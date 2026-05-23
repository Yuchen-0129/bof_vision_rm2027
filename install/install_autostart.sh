#!/usr/bin/env bash
# 安装开机自启：把 install/bof_vision.desktop.in 里的占位符替换为本仓库实际路径，
# 写到 ~/.config/autostart/bof_vision.desktop。
#
# 用法：从仓库根目录执行
#   ./install/install_autostart.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEMPLATE="${SCRIPT_DIR}/install/bof_vision.desktop.in"
TARGET_DIR="${HOME}/.config/autostart"
TARGET="${TARGET_DIR}/bof_vision.desktop"

if [[ ! -f "${TEMPLATE}" ]]; then
  echo "ERROR: template not found at ${TEMPLATE}" >&2
  exit 1
fi

if [[ ! -x "${SCRIPT_DIR}/startup.sh" ]]; then
  echo "WARN: ${SCRIPT_DIR}/startup.sh 不可执行，自动 chmod +x"
  chmod +x "${SCRIPT_DIR}/startup.sh"
fi

mkdir -p "${TARGET_DIR}"
sed "s|@SCRIPT_DIR@|${SCRIPT_DIR}|g" "${TEMPLATE}" > "${TARGET}"

echo "[OK] installed: ${TARGET}"
echo "  Exec=${SCRIPT_DIR}/startup.sh"
echo
echo "下次开机会自动启动。立即测试："
echo "  ${SCRIPT_DIR}/startup.sh"
echo "  tail -f ${SCRIPT_DIR}/logs/\$(ls -t ${SCRIPT_DIR}/logs/*.log | head -1)"
