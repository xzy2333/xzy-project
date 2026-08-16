# 功能包与命令清单

> 每一行：先看【是什么】，再看【干什么】。从上往下跑一遍就是完整流程。

## 0. 本仓库 Python 模块（不依赖 ROS，今天就能跑）

| 模块 | 是什么 | 运行命令 | 这个命令干什么 |
|---|---|---|---|
| m1_slam_sim | 手写 2D 栅格 SLAM 原理演示 | `cd ~/xzy-project/m1_slam_sim && python3 run.py` | 小车绕圈 + 激光建图，对比"只用里程计"和"扫描匹配修正"的轨迹误差，生成 `output/m1_slam_compare.png` |
| m2_waypoint_control | LOS 制导 + PID 航点控制 | `cd ~/xzy-project/m2_waypoint_control && python3 run.py` | 仿真小车按航点巡航（带洋流扰动），生成轨迹图 + 控制量图 |
| m3_path_planning | 洋流场 A* 路径规划 | `cd ~/xzy-project/m3_path_planning && python3 run.py` | 对比"最短路径 vs 最短时间路径"，生成 `output/m3_compare.png` 和指标 |
| m4_swarm | 领航-跟随编队 | `cd ~/xzy-project/m4_swarm && python3 run.py` | 仿真 3 艘跟随艇保持队形 + 避障，生成编队图 + 误差收敛曲线 |

## 1. Docker（在 20.04 上跑 ROS2 Humble 的前提）

| 命令 | 干什么 |
|---|---|
| `sudo apt install -y docker.io` | 安装 Docker |
| `sudo usermod -aG docker $USER` | 把当前用户加入 docker 组（免 sudo），注销重登生效 |
| `cd ~/xzy-project/docker && ./run.sh bash` | 首次自动构建、之后直接进入 ROS2 Humble 容器，项目目录挂载在 `/workspace` |

## 2. ROS2 功能包（在容器里安装/使用）

### 2.1 安装的包都是什么

| 包名 | 是什么 | 干什么 |
|---|---|---|
| `ros-humble-gazebo-ros-pkgs` | Gazebo 仿真器的 ROS2 接口 | 让 Gazebo 里的小车能和 ROS2 通信（传感器数据、控制指令） |
| `ros-humble-cartographer` + `cartographer-ros` | Google 开源激光 SLAM | 激光 + 里程计 → 实时栅格地图（建图） |
| `ros-humble-slam-toolbox` | 轻量 2D SLAM | 另一种建图方案，社区常用、比 Cartographer 简单 |
| `ros-humble-navigation2` + `nav2-bringup` | ROS2 导航栈 | 全局规划 + 局部避障 + 控制，把车送到目标点 |
| `ros-humble-turtlebot3-*` | TurtleBot3 机器人全家桶 | 提供仿真模型、遥控、建图、导航等（见 2.2） |

> 注：turtlebot3 的 apt 包如果装不上，就用源码编译（命令见 `ros2_car/README.md`）。

## 1.5 ROS1 主线（本机 xzy-project，已跑通）

> 指令统一用项目自定义 launch（封装官方 turtlebot3 包，对照表见 `launch/README.md`），
> 官方包名是系统 apt 依赖，不改名，只做封装。

| 本项目指令 | 封装的原指令 | 干什么 |
|---|---|---|
| `roslaunch ~/xzy-project/launch/simulation_world.launch` | `turtlebot3_gazebo turtlebot3_world.launch` | 启动 Gazebo 仿真世界 |
| `roslaunch ~/xzy-project/launch/mapping.launch` | `turtlebot3_slam turtlebot3_slam.launch` | Gmapping 建图 + RViz |
| `roslaunch ~/xzy-project/launch/teleop_keyboard.launch` | `turtlebot3_teleop turtlebot3_teleop_key.launch` | 键盘遥控（w/a/s/d） |
| `roslaunch ~/xzy-project/launch/save_map.launch` | `rosrun map_server map_saver -f ...` | 保存地图到 `maps/` |
| `roslaunch ~/xzy-project/launch/navigation.launch map_file:=$HOME/xzy-project/maps/map.yaml` | `turtlebot3_navigation turtlebot3_navigation.launch` | AMCL + move_base 导航 |

完整流程：三个终端分别跑 simulation_world → mapping → teleop_keyboard，绕完一圈跑
save_map；导航时保持 simulation_world 开着，新终端跑 navigation.launch，RViz 里先
`2D Pose Estimate` 设初始位姿（建图起点），再 `2D Nav Goal` 给目标。

### 2.2 启动与运行命令（按顺序）

| 命令 | 是什么 | 干什么 |
|---|---|---|
| `export TURTLEBOT3_MODEL=waffle_pi` | 环境变量 | 指定车型（burger 小巧 / waffle_pi 带激光+相机），**每个新终端都要设** |
| `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` | 启动仿真 | 打开 Gazebo 仿真世界，小车出现，开始发布 `/scan`、`/odom`、`/cmd_vel` |
| `ros2 run turtlebot3_teleop teleop_keyboard` | 键盘遥控 | 用键盘控制小车前后左右，验证车能动、话题通了 |
| `rviz2` | 可视化 | 打开 RViz，Add → By topic 加 `/scan`（激光）、TF（车模型），实时看数据 |
| `ros2 launch turtlebot3_cartographer cartographer.launch.py` | 建图 | 边遥控边建图，RViz 里实时看到栅格地图"长出来" |
| `ros2 run nav2_map_server map_saver_cli -f ~/map` | 保存地图 | 把建好的地图存成 `map.yaml` + `map.pgm`，供导航使用 |
| `ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=~/map.yaml` | 导航 | 加载地图启动 Nav2，RViz 里点 "2D Goal Pose"，车自动规划 + 避障走过去 |

### 2.3 诊断常用命令

| 命令 | 干什么 |
|---|---|
| `ros2 node list` / `ros2 topic list` | 看当前有哪些节点/话题，确认各包是否正常启动 |
| `ros2 topic echo /scan` | 实时打印激光数据，确认传感器通没通 |
| `rqt_graph` | 图形化看节点和话题的连接关系 |
| `ros2 run tf2_tools view_frames` | 生成 TF 坐标变换树图，排查"车在哪/传感器装在哪"的问题 |

## 3. 车 → 船迁移对照（这些包以后照样用）

| 车上 | 船上对应 |
|---|---|
| `turtlebot3_gazebo` | Stonefish 水下仿真器 |
| 2D 激光 `/scan` | 前视声呐 / 多波束 |
| Cartographer / slam_toolbox | 声呐 SLAM（原理相同） |
| Nav2 | 洋流约束 A*（m3 的 time 模式） |
| 多车命名空间 | 多艇编队 / 一致性控制 |

## 4. 十分钟速通

```bash
# 1) 先懂原理（本机直接跑）
cd ~/xzy-project/m1_slam_sim && python3 run.py

# 2) 进 ROS2 容器
cd ~/xzy-project/docker && ./run.sh bash

# 3) 容器内：仿真 + 建图 + 遥控 + 可视化（四个终端，都要先 export）
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py
ros2 run turtlebot3_teleop teleop_keyboard
rviz2
```
