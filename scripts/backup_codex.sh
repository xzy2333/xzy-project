#!/usr/bin/env bash
set -euo pipefail

# 备份 Codex 完整数据（会话历史 / skills / rules / 配置），重装系统后恢复用
# 输出：~/codex_backup-<日期>.tar.gz
# 注意：config.toml 内含 API Key，压缩包请自己保管好，不要传到公开仓库
OUT=~/codex_backup-$(date +%Y%m%d-%H%M%S).tar.gz

if [ -d ~/.codex ]; then
  tar -czf "$OUT" -C ~ .codex
  echo "已备份到 $OUT ($(du -h "$OUT" | cut -f1))"
else
  echo "未找到 ~/.codex，跳过" >&2
  exit 1
fi

