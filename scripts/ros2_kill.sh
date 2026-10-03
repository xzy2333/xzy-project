#!/usr/bin/env bash
# 在容器里运行：关掉卡住的 ROS2 仿真 / 建图 / 导航 / 遥控进程
#
# 为什么需要它：nav2_bringup 启的是 component_container_isolated，它对 SIGINT 反应很慢
# ——实测按 Ctrl+C 后要 5 秒才升级 SIGTERM、再 5 秒升级 SIGKILL，总共约 15 秒才退。
# 等不及就在**另一个终端**里跑这个脚本，或者直接对 launch 进程 Ctrl+C 后耐心等 15 秒。
#
# 用法（容器内）：bash /workspace/scripts/ros2_kill.sh
# 只按精确进程名和固定字符串匹配，不用 pkill -f（那个会匹配到自己的命令行，本项目踩过）
set -u

snapshot="$(ps -eo pid,args --no-headers)"

kill_line() {
  local pid="$1" args="$2"
  echo "  杀 $pid : $args"
  kill -9 "$pid" 2>/dev/null || true
}

echo "== 1) 按命令行匹配（ros2 launch / 键盘遥控）=="
while read -r pid args; do
  case "$args" in
    *"/opt/ros/humble/bin/ros2 launch"*) kill_line "$pid" "$args" ;;
    *"turtlebot3_teleop/teleop_keyboard"*) kill_line "$pid" "$args" ;;
  esac
done <<< "$snapshot"

echo "== 2) 按精确进程名匹配 =="
for n in component_container_isolated rviz2 cartographer_node \
         cartographer_occupancy_grid_node gzserver gzclient; do
  for p in $(pgrep -x "$n" 2>/dev/null); do
    kill_line "$p" "$n"
  done
done

sleep 2
echo "== 剩余的相关进程 =="
if ps -eo pid,args --no-headers | grep -E "[r]os2 launch|[c]omponent_container|[r]viz2" ; then
  echo "(上面还有残留，再跑一次本脚本)"
else
  echo "(干净了)"
fi
