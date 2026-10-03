#!/usr/bin/env bash
# 在容器里运行：把当前 cartographer 正在发布的地图存到 maps/ 下
#
# 用法（进容器后，cartographer 必须还在跑）：
#   bash /workspace/scripts/ros2_save_map.sh              # 默认存成 ros2_xzy_lab
#   bash /workspace/scripts/ros2_save_map.sh ros2_lab     # 指定名字
#
# 两个细节：
#   1. 必须存到 /workspace/maps/ 下：容器里的 ~ 是 /root，而容器是 --rm，退出即销毁；
#   2. 阈值用 free_thresh_default:=0.196，和 ROS1 那边 map_saver 的口径对齐
#      （Humble 这版参数名是 *_default，写 free_thresh 不生效）；
#   3. 存完把属主改回挂载目录的属主，免得宿主机上是 root/nobody 写不进去。
set -e
source /opt/ros/humble/setup.bash

name="${1:-ros2_xzy_lab}"
out="/workspace/maps/${name}"

ros2 run nav2_map_server map_saver_cli -f "$out" --ros-args \
  -p save_map_timeout:=30.0 \
  -p free_thresh_default:=0.196 \
  -p occupied_thresh_default:=0.65

owner="$(stat -c '%u:%g' /workspace)"
chown "$owner" "$out.pgm" "$out.yaml" 2>/dev/null || true
ls -l "$out.pgm" "$out.yaml"
