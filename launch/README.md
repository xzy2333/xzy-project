# 自定义 launch（xzy-project）

本目录把官方 TurtleBot3 的启动命令封装成项目自己的名字，指令统一走 `roslaunch` + 绝对路径或 `roslaunch ~/xzy-project/launch/xxx.launch`：

| 本项目命令 | 封装的原命令 | 作用 |
|---|---|---|
| `simulation_world.launch` | `turtlebot3_gazebo/turtlebot3_world.launch` | 启动 Gazebo 仿真世界 |
| `mapping.launch` | `turtlebot3_slam/turtlebot3_slam.launch` | Gmapping 建图 + RViz |
| `teleop_keyboard.launch` | `turtlebot3_teleop/turtlebot3_teleop_key.launch` | 键盘遥控 |
| `navigation.launch` | `turtlebot3_navigation/turtlebot3_navigation.launch` | AMCL 定位 + move_base 导航 |
| `save_map.launch` | `rosrun map_server map_saver -f ...` | 保存地图到 `maps/` |

用法：

```bash
roslaunch ~/xzy-project/launch/simulation_world.launch
roslaunch ~/xzy-project/launch/mapping.launch
roslaunch ~/xzy-project/launch/teleop_keyboard.launch
roslaunch ~/xzy-project/launch/save_map.launch
roslaunch ~/xzy-project/launch/navigation.launch            # 默认加载 maps/map.yaml
roslaunch ~/xzy-project/launch/navigation.launch map_file:=$HOME/xzy-project/maps/map.yaml
roslaunch ~/xzy-project/launch/simulation_world.launch world_file:=$HOME/xzy-project/worlds/my_world.world  # 自建世界
```

注意：

- 底层依赖的官方包（`turtlebot3_gazebo` 等）是 apt 安装的系统包，**不能改名**——改了会破坏
  turtlebot3_slam / turtlebot3_navigation 对它的依赖和 apt 完整性。这里用 launch 封装达到
  "项目自己的指令名"的效果，更安全也更整洁。
- 每个终端仍要 `export TURTLEBOT3_MODEL=waffle_pi`（封装里已固定 model，不设也能跑，但设了保险）。
