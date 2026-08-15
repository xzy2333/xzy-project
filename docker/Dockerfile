# ROS2 Humble 容器（在 Ubuntu 20.04 主机上用 Docker 运行）
FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-numpy python3-matplotlib git vim \
    && rm -rf /var/lib/apt/lists/*

# 后续需要 Stonefish 时，参考 https://github.com/patrykcieslak/stonefish 的 ROS2 安装说明
CMD ["/bin/bash"]

