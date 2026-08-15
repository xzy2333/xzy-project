# uuv_navi —— 面向哈工程青岛基地（闫金金老师方向）的仿真项目

目标：按"先车后船"主线，用纯电脑仿真把无人系统的四层关键技术（**感知→规划→控制→协同**）先在小车上跑通，再逐个迁移到水面/水下。

逻辑：算法与载体无关——小车、无人机、船用的是同一套 ROS2/SLAM/规划/控制栈，变的只是模型、动力学、传感器。所以先用小车把闭环跑通（Gazebo + RViz 可视化），再把同一套技术迁移到船/UUV（换 URDF、换 Stonefish、加洋流/声呐约束）。

## 目录结构

```
uuv_navi/
├── CHECKLIST.md              # 技术栈清单（勾选用，最重要）
├── m1_slam_sim/              # 感知：手写 2D 栅格 SLAM（先懂原理）
├── m2_waypoint_control/      # 控制：LOS 制导 + PID 航点跟踪
├── m3_path_planning/         # 规划：A*（洋流版本留给船）
├── m4_swarm/                 # 协同：领航-跟随编队 + 避碰
├── ros2_car/                 # 车上跑 ROS2+Gazebo+RViz 的教程（turtlebot3）
├── docker/                   # ROS2 Humble 容器（20.04 上用 Docker 跑）
├── maps/                     # ROS1 主线建图保存的地图（map.yaml + map.pgm）
├── blog/                     # 技术博客初稿 + 演示视频录制清单（复试素材）
├── docs/                     # 环境恢复指南 + 系统快照（重装系统后恢复用）
├── scripts/                  # 环境备份 / 恢复 / Codex 备份脚本
└── README.md
```

> 要清理硬盘重装系统？先看 [docs/ENV_RESTORE.md](docs/ENV_RESTORE.md)，运行 `scripts/backup_env.sh` 抓取环境快照，重装后按指南一键恢复。

## 快速开始（不需要 ROS，今天就能跑）

```bash
cd ~/uuv_navi/m1_slam_sim
python3 run.py        # 生成 output/m1_slam_compare.png（SLAM 原理演示）

cd ~/uuv_navi/m2_waypoint_control
python3 run.py        # 生成 output/m2_trajectory.png 和 m2_control.png

cd ~/uuv_navi/m3_path_planning
python3 run.py        # 生成 output/m3_compare.png

cd ~/uuv_navi/m4_swarm
python3 run.py        # 生成 output/m4_formation.png
```

依赖：Python 3.8 + numpy + matplotlib（本机已具备）。

**每个包和命令是干什么的，见 [COMMANDS.md](COMMANDS.md)（功能包与命令清单）。**

## 时间线

| 时间 | 任务 |
|---|---|
| 8~9 月 | 跑通 m1（SLAM 原理）、m2（控制）、m3（规划），读闫老师论文做笔记 |
| 10 月中前 | 按 ros2_car/README 跑通 turtlebot3：Gazebo 运动 + Cartographer 建图 + RViz |
| 10 月中~12 月 | 项目冻结，全力初试 |
| 12 月考完~2 月 | m4 编队、Nav2 导航、博客 + 视频 + 简历 |
| 出分后 | 邮件联系闫老师，附 GitHub + 视频 |

## ROS2 怎么用（Ubuntu 20.04）

> 本机已装 ROS1 Noetic（`/opt/ros/noetic` + Gazebo 11）。**先用 ROS1 跑通主流程**（见 [ENV_SETUP.md](ENV_SETUP.md) 第 4 节）；ROS2 用 Docker 跑 Humble，作为集群/多机阶段的备用环境：

```bash
sudo apt update && sudo apt install -y docker.io
sudo usermod -aG docker $USER   # 然后注销重登
cd ~/uuv_navi/docker
./run.sh bash                   # 进入 ROS2 Humble 容器
```

容器里已经预装 ROS2 Humble 桌面版 + Python 工具。Stonefish 水下仿真器等用到时再装。

## 下一步

1. 打开 CHECKLIST.md，先勾 m1/m2/m3 三项并跑通
2. 每跑通一个模块，写一篇博客（这是复试素材）
3. ROS1 主线博客初稿已就位：`blog/01_gazebo_slam_navigation.md`；录视频前看 `blog/VIDEO_CHECKLIST.md`
4. 有问题随时把报错贴给我
