#!/usr/bin/env bash
# gmapping 参数对照实验：依次用 A/B/C 三组参数建同一张地图，结果存到
# maps/lab_<组名>.{pgm,yaml}，并打印各组用时（地图质量用 map_quality.py 另算）。
#
# 用法：bash ~/xzy-project/scripts/gmapping_param_sweep.sh
# 前置：没有别的 Gazebo / rosmaster 在跑（脚本会先检查，有则直接退出）。
set -u

WS="$HOME/xzy-project"
source /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

if pgrep -x gzserver >/dev/null || pgrep -x rosmaster >/dev/null; then
  echo "检测到已有 gzserver/rosmaster 在运行，请先关闭它们再跑本脚本。" >&2
  exit 1
fi

for g in A_official B_fine C_lenient; do
  echo "======== 参数组 $g ========"
  roslaunch "$WS/launch/simulation_world.launch" \
      world_file:="$WS/worlds/xzy_lab.world" gui:=false > "/tmp/sweep_${g}_world.log" 2>&1 &
  W_PID=$!
  sleep 14

  roslaunch "$WS/launch/mapping.launch" open_rviz:=false \
      gmapping_params:="$WS/launch/gmapping_params_${g}.yaml" \
      > "/tmp/sweep_${g}_mapping.log" 2>&1 &
  M_PID=$!
  sleep 10

  T0=$(date +%s)
  python3 "$WS/scripts/drive_lab.py" > "/tmp/sweep_${g}_drive.log" 2>&1
  T1=$(date +%s)
  sleep 2
  rosrun map_server map_saver -f "$WS/maps/lab_${g}" > "/tmp/sweep_${g}_save.log" 2>&1

  echo "参数组 $g：路线耗时 $((T1 - T0)) s；地图 -> maps/lab_${g}.pgm"
  glow_log="/tmp/sweep_${g}_mapping.log"
  echo "参数组 $g：处理扫描帧 $(rg -c "update frame" "$glow_log" 2>/dev/null || echo 0)，" \
       "匹配失败/丢弃告警 $(rg -ci "dropped|failed" "$glow_log" 2>/dev/null || echo 0)"

  kill $M_PID $W_PID 2>/dev/null
  sleep 8
done

echo "全部完成：对每组地图跑 python3 $WS/scripts/map_quality.py 得到 IoU"
