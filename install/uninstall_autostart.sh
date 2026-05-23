#!/usr/bin/env bash
# 卸载开机自启：删除 ~/.config/autostart/bof_vision.desktop。
# 不会停掉已经在跑的进程，要停的话另外 kill -- 或重启。

set -euo pipefail

TARGET="${HOME}/.config/autostart/bof_vision.desktop"

if [[ -f "${TARGET}" ]]; then
  rm -f "${TARGET}"
  echo "[OK] removed: ${TARGET}"
else
  echo "[skip] ${TARGET} not found"
fi
