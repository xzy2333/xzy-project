#!/usr/bin/env bash
set -euo pipefail

# 在重装后的 Ubuntu 20.04 上恢复 ROS 相关环境（基于 ~/env_backup/ 快照）
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SNAP="$HOME/env_backup"

if [ ! -f "$SNAP/apt_packages.txt" ]; then
  echo "找不到快照 $SNAP，请先确认仓库完整" >&2
  exit 1
fi

echo "==> 安装 ROS Noetic 相关包（来自快照）"
grep -E '^ros-noetic-' "$SNAP/apt_packages.txt" | xargs sudo apt-get install -y

echo "==> 安装 Gazebo 与 Python 依赖"
sudo apt-get install -y gazebo11 python3-numpy python3-matplotlib

echo "==> 追加 bashrc 片段（幂等）"
if [ -f "$SNAP/bashrc_snippet.txt" ]; then
  while IFS= read -r line; do
    grep -qF -- "$line" ~/.bashrc || echo "$line" >> ~/.bashrc
  done < "$SNAP/bashrc_snippet.txt"
fi

echo "==> 完成。"
echo "仍需手动处理："
echo "  1. /opt/MVS（海康 SDK，官网下载）"
echo "  2. ~/.codex/config.toml 的 API Key"
echo "  3. GitHub SSH 公钥"
