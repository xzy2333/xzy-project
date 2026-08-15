#!/usr/bin/env bash
set -euo pipefail

# 抓取当前系统环境快照，写入 docs/system_backup/，重装系统后用于恢复
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$REPO_DIR/docs/system_backup"
mkdir -p "$OUT"

# 已安装的 apt 包（完整列表）
dpkg -l | awk '/^ii/{print $2}' > "$OUT/apt_packages.txt"

# 手动安装的 apt 包（更精简的清单）
apt-mark showmanual > "$OUT/apt_manual_packages.txt" 2>/dev/null || true

# Python 包
pip3 list --format=freeze > "$OUT/pip_packages.txt" 2>/dev/null || pip3 freeze > "$OUT/pip_packages.txt"

# 系统版本
head -2 /etc/os-release > "$OUT/os_version.txt"

# bashrc 中与 ROS/MVS/TurtleBot3 相关的行（可直接追加回 .bashrc）
grep -E 'TURTLEBOT3_MODEL|MVCAM|MVS|source /opt/ros' ~/.bashrc > "$OUT/bashrc_snippet.txt" || true

# Codex 配置（API Key 脱敏）与模型目录
if [ -f ~/.codex/config.toml ]; then
  sed -E 's/experimental_bearer_token[[:space:]]*=[[:space:]]*"[^"]*"/experimental_bearer_token = "<REDACTED>"/' \
    ~/.codex/config.toml > "$OUT/codex_config.toml.example"
fi
cp ~/.codex/models.json "$OUT/codex_models.json" 2>/dev/null || true
codex --version > "$OUT/codex_version.txt" 2>/dev/null || true

echo "环境快照已写入 $OUT"

