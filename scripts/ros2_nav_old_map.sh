#!/usr/bin/env bash
# 在容器里运行：用 ROS1 建的老图（maps/xzy_lab.yaml）跑 Nav2 导航
#
# 用法（进容器后）：
#   bash /workspace/scripts/ros2_nav_old_map.sh                 # 带 RViz
#   bash /workspace/scripts/ros2_nav_old_map.sh open_rviz:=false
#
# 为什么初始位姿是 (-2.0, -0.5)：ROS1/gmapping 的 map 原点 ≈ odom 原点，建图起点
# 落在 map(-2.0, -0.5)；而 cartographer 的 map 原点是建图起点，即 map(0,0)。
# 用老图就要给老图的坐标，否则 AMCL 会从错的位置起算。
set -e
source /opt/ros/humble/setup.bash

ros2 launch /workspace/launch/ros2/navigation.launch.py \
  map_file:=/workspace/maps/xzy_lab.yaml \
  initial_pose_x:=-2.0 \
  initial_pose_y:=-0.5 \
  initial_pose_a:=0.0 \
  "$@"
