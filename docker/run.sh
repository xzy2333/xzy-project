#!/usr/bin/env bash
# 用法：./run.sh bash   （第一次运行会自动构建镜像，约需下载 3~5 GB）
set -e
IMAGE=uuv_ros2_humble

# 项目根目录 = 本脚本所在目录的上一级（从任何路径调用都能挂对）
DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$DOCKER_DIR")"

# 没进 docker 组时给一句人话提示，别让人对着 permission denied 发呆
if ! docker info >/dev/null 2>&1; then
  if sg docker -c 'docker info' >/dev/null 2>&1; then
    echo "[提示] 当前 shell 不在 docker 组里，用 sg docker 转发（重新登录后就不需要了）……"
    exec sg docker -c "$(printf '%q ' "$0" "$@")"
  fi
  echo "[错误] 连不上 docker。先确认：systemctl is-active docker；groups | grep docker（新加的组要重新登录才生效）" >&2
  exit 1
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[提示] 本地没有 $IMAGE，先构建（约 5~10 分钟，1~2 GB 下载）……"
  docker build -t "$IMAGE" "$DOCKER_DIR"
fi

# 允许容器显示图形界面（Linux 本地显示）
xhost +local: >/dev/null 2>&1 || true

# 交互终端才加 -it；被脚本调用（无 TTY）时不加，避免 "the input device is not a TTY"
TTY_FLAG=()
if [ -t 0 ] && [ -t 1 ]; then TTY_FLAG=(-it); fi

# --net=host 只解决"发现"，不解决"传数据"：不加 --ipc=host 时各容器 /dev/shm 独立，
# DDS 走共享内存取不到对方的段，表现为"话题列表看得见、但一条数据都收不到"
# （实测：两个容器互发 /probe，默认配置收不到；双方加 --ipc=host 后收到 2.000 Hz）。
# 所以 Gazebo 一个容器、cartographer/RViz 一个容器时，必须带上它。
docker run "${TTY_FLAG[@]}" --rm \
  --net=host \
  --ipc=host \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PROJECT_ROOT":/workspace \
  -w /workspace \
  "$IMAGE" "$@"
