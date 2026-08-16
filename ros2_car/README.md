# 车（UGV）先行：ROS2 + Gazebo + RViz 实战路线

> 为什么先做车：你的专业是无人系统方向，做车名正言顺；算法栈（ROS2、SLAM、规划、控制、集群）和船/UUV 完全一样，变的只是**模型、动力学、传感器**。先车后船 = 先把通用技术验证清楚，再迁移到目标载体。

> **注意：本机已装 ROS1 Noetic。想先用 ROS1 一条龙跑通（更省事），看 [ENV_SETUP.md](../ENV_SETUP.md) 第 4 节；本文件是 ROS2 版，等集群/多机阶段再用 Docker 容器跑。**

前提：Docker 装好（见 [README](../README.md)），进入容器：

```bash
cd ~/xzy-project/docker && ./run.sh bash
```

## A1：Gazebo 里让小车动起来

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-cartographer \
    ros-humble-cartographer-ros ros-humble-slam-toolbox ros-humble-navigation2 \
    ros-humble-nav2-bringup
# TurtleBot3（若 apt 找不到包，改用下面的源码编译方式）
sudo apt-get install -y ros-humble-turtlebot3-*
# 源码方式（apt 装不上时用）：
#   mkdir -p ~/tb3/src && cd ~/tb3/src
#   git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
#   git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
#   cd ~/tb3 && colcon build --symlink-install
#   echo "source ~/tb3/install/setup.bash" >> ~/.bashrc && source ~/.bashrc

export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

新开一个终端（同样进容器），键盘控制小车：

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```

同时开 RViz 看模型和激光（新终端）：

```bash
export TURTLEBOT3_MODEL=waffle_pi
rviz2
# 左下 Add → By topic → /scan (LaserScan) 和 /odom (TF) → 小车在激光里出现了
```

> 无独立显卡的机器，先 `export LIBGL_ALWAYS_SOFTWARE=1` 再启动 Gazebo（软件渲染，慢但能跑）。

## A2：SLAM 建图 + RViz 实时显示

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

用键盘把车开遍地图，RViz 里会看到栅格地图一点点被构建出来——这就是你刚在 [m1_slam_sim](../m1_slam_sim/README.md) 里手写的东西的"工业版"。

建图完成保存：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## A3：路径规划（Nav2）

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=~/map.yaml
```

RViz 里用 "2D Goal Pose" 点一个目标，小车自动规划路径并导航过去。Nav2 的全局规划器干的事，和你 [m3](../m3_path_planning/README.md) 里写的 A* 是同一个逻辑。

## A4：多车（集群的工程版，考完研后做）

- 用多命名空间启动多台 TurtleBot3 仿真，每台跑一套 slam_toolbox
- 把你 [m4](../m4_swarm/README.md) 的编队算法包成 ROS2 节点：订阅各车位姿（`/robotX/odom`），计算队形指令，发布到各车 `/robotX/cmd_vel`
- 这一步就是"集群"从 Python 动画变成真 ROS2 系统的过程

## 车 → 船迁移对照（技术栈不变，只换载体）

| 层 | 车上 | 船/潜航器上 |
|---|---|---|
| 模型 | TurtleBot3 / 自写 URDF | USV/UUV URDF + Stonefish |
| 里程计 | Gazebo 差速轮 | DVL + IMU |
| 感知 | 2D 激光 | 前视声呐 / 水下相机 |
| SLAM | Cartographer / slam_toolbox | 声呐 SLAM（原理相同） |
| 规划 | Nav2 / A* | 洋流约束 A*（m3 的 time 模式） |
| 集群 | 多车命名空间 | 多艇一致性 / 编队 |

复试话术参考："我本科方向是无人系统，先在地面小车把建图、导航、编队验证清楚，后续想拓展到水面/水下无人系统，所以研究了您这边的课题。"
