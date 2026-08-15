#!/usr/bin/env bash
# 用法：./run.sh bash   （第一次运行会自动构建镜像，约需下载 3~5 GB）
set -e
IMAGE=uuv_ros2_humble

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  docker build -t "$IMAGE" .
fi

# 允许容器显示图形界面（Linux 本地显示）
xhost +local: >/dev/null 2>&1 || true

docker run -it --rm \
  --net=host \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(dirname "$PWD")":/workspace \
  -w /workspace \
  "$IMAGE" "$@"

