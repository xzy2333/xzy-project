# ROS2 Humble 容器（在 Ubuntu 20.04 主机上用 Docker 运行）
FROM osrf/ros:humble-desktop

# 换阿里云 ubuntu 源（国内速度），ROS2 源保持 packages.ros.org
RUN sed -i -e 's@//archive.ubuntu.com/ubuntu@//mirrors.aliyun.com/ubuntu@g' -e 's@//security.ubuntu.com/ubuntu@//mirrors.aliyun.com/ubuntu@g' /etc/apt/sources.list

# ROS2 源换 TUNA 镜像（官方源实测 ~3 KB/s，TUNA 响应 0.15 s）
# 去掉 deb-src：TUNA 没有 source 索引（404），我们只装二进制包
RUN sed -i -e 's@http://packages.ros.org/ros2/ubuntu@https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu@g' \
           -e 's@^Types: deb deb-src$@Types: deb@' \
           /etc/apt/sources.list.d/ros2.sources \
    && grep -h -e URIs -e Types /etc/apt/sources.list.d/ros2.sources

# 基础工具 + 仿真/SLAM/导航全家桶
#   turtlebot3-*     ：小车模型、仿真世界、键盘遥控、SLAM/导航 launch
#   gazebo-ros-pkgs  ：gazebo_ros（Gazebo Classic 11）插件
#   cartographer-ros ：建图（对应 ROS1 的 gmapping 位置）
#   navigation2/nav2 ：Nav2 导航栈（对应 ROS1 的 move_base）
#   slam-toolbox     ：另一种 2D SLAM，多车阶段会用到
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-numpy python3-matplotlib git vim tmux curl \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-teleop \
    ros-humble-turtlebot3-cartographer \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# 车型与渲染：本机无独显，Gazebo 走软件渲染（LLVMpipe），慢但能跑
# GAZEBO_MODEL_PATH 必须包含 /usr/share/gazebo-11/models：否则 ground_plane/sun
# 找不到本地模型，Gazebo 会去 models.gazebosim.org 联网下载（国内被墙 → 卡死在
# "Loading world file"，表现为 /spawn_entity 服务永不出现、spawn 失败）
ENV TURTLEBOT3_MODEL=waffle_pi \
    GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models \
    LIBGL_ALWAYS_SOFTWARE=1 \
    RCUTILS_COLORIZED_OUTPUT=1 \
    PYTHONDONTWRITEBYTECODE=1

# 后续需要 Stonefish 时，参考 https://github.com/patrykcieslak/stonefish 的 ROS2 安装说明
CMD ["/bin/bash"]
