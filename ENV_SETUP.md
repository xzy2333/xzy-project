# 环境配置与 ROS 选型（ROS1 vs ROS2）

## 0. 结论

- 本机**已经是 ROS1 Noetic + Gazebo 11 环境**（实测 `/opt/ros/noetic`、Gazebo 11.15.1）
- 主线：**先用 ROS1 Noetic**（零成本、水下生态多、教程最全），**ROS2 Humble 用 Docker 备用**（集群/多机阶段再启用，方案已备好）
- 最终以导师实验室实际环境为准（确认方法见第 3 节）

## 1. 本机现状（2026-08-09 检测）

| 项目 | 结果 |
|---|---|
| Ubuntu 20.04.6 LTS | ✓ |
| ROS1 Noetic（desktop） | ✓ 已装 `/opt/ros/noetic` |
| Gazebo | ✓ 11.15.1 |
| turtlebot3 / SLAM 包 | ✗ 未装（下一步装） |
| Docker | ✗ 未装 |
| NVIDIA GPU | ✗ 无（深度学习模块去 AutoDL 云端） |
| 磁盘 / 内存 | 513G 空闲 / 16 核 15G |

## 2. ROS1 和 ROS2 到底差在哪

| 维度 | ROS1 (Noetic) | ROS2 (Humble) |
|---|---|---|
| 通讯架构 | 中心化：`roscore` 注册中心 | DDS 分布式，无中心 |
| 多机/多机器人 | 手动配 master，麻烦 | 原生支持 → 集群方向的主场 |
| 语言 | Noetic 支持 Python3（早期 ROS1 是 Py2） | Python3 + C++17 |
| 命令 | `rosrun` / `roslaunch` | `ros2 run` / `ros2 launch` |
| 经典包 | gmapping、move_base、UUV Simulator | Nav2、slam_toolbox、Stonefish |
| 维护状态 | 已停止维护（2025-05 EOL） | Humble LTS 到 2027 |
| 中文资料 | 最全 | 快速增长 |
| 概念 | 节点/话题/服务/参数/TF | 相同 + QoS/生命周期 |

**结论：概念 90% 相通，换版本主要是命令和架构差异。先学 Noetic 完全不吃亏。**

## 3. 导师环境怎么确认（复试前完成）

1. 微信公众号「先进智能导航实验室」往期招新/技术推文
2. B 站「AI Navi Lab」视频简介与评论
3. 复试前问师兄（最直接）：
   - "咱们机器人端现在主要用 ROS1 还是 ROS2？"
   - "平时仿真用 Gazebo 还是自研平台？"

记录：导师环境 = ________（填）→ 我的主线 = ________

## 4. 主线：ROS1 Noetic（本机）

### 4.1 安装缺的包

```bash
sudo apt update
sudo apt install -y ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations \
    ros-noetic-navigation ros-noetic-gmapping
```

### 4.2 跑通"车仿真 + SLAM 建图 + RViz"（三个终端）

```bash
# 终端 1：仿真世界
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# 终端 2：SLAM 建图（默认 gmapping，会自动打开 RViz）
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_slam turtlebot3_slam.launch

# 终端 3：键盘遥控，边开边建图
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

建图完成保存：

```bash
rosrun map_server map_saver -f ~/map
```

导航（RViz 里点 2D Nav Goal）：

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

### 4.3 关于 Cartographer

ROS1 Noetic 的 Cartographer **没有二进制包**，要源码编译（比较折腾）。
建议：先用 Gmapping 跑通（2D 激光 SLAM 原理一样）；以后想上 Cartographer 直接去 ROS2 容器（有现成二进制）。

### 4.4 验证命令

```bash
source /opt/ros/noetic/setup.bash
rosversion -d                 # 显示 noetic
rospack find turtlebot3_gazebo   # 显示路径说明装好了
```

## 5. 备用：ROS2 Humble（Docker）

- 容器配置已写好：`cd ~/uuv_navi/docker && ./run.sh bash`
- 用途：Nav2、slam_toolbox、Stonefish、多车集群——ROS2 的主场
- 注意：**ROS1 和 ROS2 不要在同一终端同时 source**（本机终端默认已 source Noetic，进容器后是独立的）

## 6. 常见坑

- 每个终端都要 `export TURTLEBOT3_MODEL=waffle_pi`（想省事可写进 `~/.bashrc`）
- 本机终端已自动 source Noetic（`ROS_DISTRO=noetic`），不用手动 source
- 以后接真机做多机时，ROS1 要配 `ROS_MASTER_URI`
- 想用 Cartographer：Noetic 先用 Gmapping 替代，或直接上 ROS2 容器

