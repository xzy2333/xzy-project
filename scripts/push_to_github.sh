#!/usr/bin/env bash
set -euo pipefail

# 用法：bash scripts/push_to_github.sh <仓库地址>
# 例：  bash scripts/push_to_github.sh https://github.com/<你的用户名>/xzy-project.git
# 或：  bash scripts/push_to_github.sh git@github.com:<你的用户名>/xzy-project.git
REPO_URL="${1:?请提供 GitHub 仓库地址，例如 https://github.com/you/xzy-project.git}"

cd "$(dirname "${BASH_SOURCE[0]}")/.."

git remote remove origin 2>/dev/null || true
git remote add origin "$REPO_URL"
git push -u origin main

echo "推送完成"
