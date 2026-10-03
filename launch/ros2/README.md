# ROS2 Humble launch（容器里跑）

本目录是 **ROS2 版**的启动文件，与上一层 `launch/*.launch`（ROS1 Noetic）一一对应。
ROS2 Humble 只能跑在 Docker 容器里（本机是 Ubuntu 20.04），所以下面所有命令都要先
`cd ~/xzy-project/docker && ./run.sh bash` 进容器，**每个终端都要进一次**。

容器里项目根目录是 `/workspace`（挂载的就是宿主机的 `~/xzy-project`）。

## 文件对照

| 本目录（ROS2） | 上一层（ROS1） | 封装的原命令 | 作用 |
|---|---|---|---|
| `sim_xzy_lab.launch.py` | `simulation_world.launch` | `gazebo_ros/gzserver.launch.py` + `turtlebot3_gazebo/spawn_turtlebot3.launch.py` | 加载 `worlds/xzy_lab.world`，在 (-2.0, -0.5) 生成 waffle_pi |
| `mapping.launch.py` | `mapping.launch` | `turtlebot3_cartographer/cartographer.launch.py` | Cartographer 建图（ROS2 里对应 gmapping 的位置） |
| `navigation.launch.py` | `navigation.launch` | `nav2_bringup/bringup_launch.py` | Nav2 导航 + **自动给 AMCL 初始位姿** |

## 完整流程（照着敲）

```bash
# 终端 1：仿真（gui:=false 是无界面，自检用；平时不加）
ros2 launch /workspace/launch/ros2/sim_xzy_lab.launch.py

# 终端 2：键盘遥控
ros2 run turtlebot3_teleop teleop_keyboard

# 终端 3：建图（RViz 会跟着起来）
ros2 launch /workspace/launch/ros2/mapping.launch.py

# 终端 4：存图（cartographer 必须还活着！存到 /workspace 才不会被容器销毁）
ros2 run nav2_map_server map_saver_cli -f /workspace/maps/ros2_xzy_lab \
  --ros-args -p save_map_timeout:=30.0 \
             -p free_thresh_default:=0.196 \
             -p occupied_thresh_default:=0.65

# 终端 5：导航（车停在建图起点时，默认初始位姿 0,0,0）
ros2 launch /workspace/launch/ros2/navigation.launch.py \
    map_file:=/workspace/maps/ros2_xzy_lab.yaml

# 6. 设目标点：RViz 里点 "2D Goal Pose"，或命令行发（示例目标 map(1.0, 0.0)）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

## 参数

`sim_xzy_lab.launch.py`：`world_file`（默认 `worlds/xzy_lab.world`）、`x_pos`/`y_pos`
（默认 -2.0 / -0.5，和建图起点一致）、`gui`、`use_sim_time`。

`mapping.launch.py`：`use_sim_time`（默认 true，仿真必须）、`open_rviz`、
`configuration_basename`。

`navigation.launch.py`：`map_file`、`open_rviz`、`use_sim_time`、`params_file`、
`initial_pose_x` / `initial_pose_y` / `initial_pose_a`（默认 0 / 0 / 0，即建图起点）。

## 踩过的坑（都是实测，不是推测）

1. **跨容器必须 `--ipc=host`**：只有 `--net=host` 时话题列表看得见、数据一条收不到
   （两个容器互发 `/probe` 实测：默认 0 条，加 `--ipc=host` 后 2.000 Hz）。
   `docker/run.sh` 已经带上，别自己手敲 `docker run` 时漏掉。
2. **存图必须存到 `/workspace/` 下**：容器里的 `~` 是 `/root`，而容器是 `--rm`，退出即销毁。
3. **阈值参数名**：Humble 这版是 `free_thresh_default` / `occupied_thresh_default`；
   写 `free_thresh` 不生效（实测 yaml 里还是默认 0.25）。0.196 是为了和 ROS1 那边
   `map_saver` 的口径对齐。
4. **ROS2 官方导航 launch 不给初始位姿**（`turtlebot3_navigation2/launch/navigation2.launch.py`
   里没有任何 initial 字样），不给 AMCL 初始位姿它就不发 `map→odom`，Nav2 一个目标都走不了。
   本目录的 `navigation.launch.py` 在启动 12 秒后自动连发 3 次 `/initialpose` 补上。
5. **Gazebo 模型路径**：镜像里已设 `GAZEBO_MODEL_PATH` 含 `/usr/share/gazebo-11/models`。
   少了它，`ground_plane`/`sun` 找不到本地模型会去联网下载，国内直接卡死在
   "Loading world file"（表现：`/spawn_entity` 服务永不出现、spawn 失败）。
6. **`__pycache__` 属主**：容器里以 root 跑 `ros2 launch` 会在仓库里生成 root 属主的
   `__pycache__/`，宿主机这边就写不进去了。已在 `.gitignore` 里，要删用
   `pkexec rm -rf launch/ros2/__pycache__`。

## 验证状态（重要）

- `sim_xzy_lab.launch.py`：**已在容器里实测通过**——自建世界加载、车生成在 (-2.0, -0.5)、
  `/scan` 4.98 Hz、`/odom` `/tf` `/cmd_vel` `/clock` 话题齐全、日志无报错。
- `mapping.launch.py` / `navigation.launch.py`：只做了 `--show-args` 静态加载验证
  （参数能解析、默认路径正确），**没有跑完整流程**。建图与导航的底层命令在官方
  `turtlebot3_cartographer` / `nav2_bringup` 上实测过（见 `docs/` 与本文档的坑清单），
  但这一组文件本身还没端到端验证。
