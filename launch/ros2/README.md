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
（默认 -2.0 / -0.5，和建图起点一致）、`gui`、`use_sim_time`、`robot_sdf`
（默认仓库里的精简模型 `worlds/models/turtlebot3_waffle_pi/model.sdf`，见该目录 README；
想跑官方完整模型对比就把它指回 `/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf`）。

`mapping.launch.py`：`use_sim_time`（默认 true，仿真必须）、`open_rviz`、
`configuration_basename`。

`navigation.launch.py`：`map_file`、`open_rviz`、`use_sim_time`、`params_file`、
`rviz_config_file`、`initial_pose_x` / `initial_pose_y` / `initial_pose_a`
（默认 0 / 0 / 0，即建图起点）。`open_rviz:=false` 可不起 RViz。

## 用 ROS1 建的老图跑 ROS2 导航（不用重建）

栅格地图是跟 ROS 版本无关的中间表示（pgm + yaml），ROS1 的 `map_server` 和 ROS2 的
`nav2_map_server` 读的是同一套格式。老图 `maps/xzy_lab.*` 是在同一个
`worlds/xzy_lab.world`、同一个 spawn 点建的，几何直接对得上。**已实测**：容器里加载
`/workspace/maps/xzy_lab.yaml`，`/map` 分辨率 0.05、384×384，与 ROS1 那边一致。

但有两处"约定"要处理，否则会失败：

**1）yaml 里的 `image:` 必须是相对路径。** ROS1 的 `map_saver` 会写成
`/home/xzy/xzy-project/maps/xzy_lab.pgm` 这种宿主机绝对路径，容器里没有 `/home/xzy`
这个目录，加载直接失败。已把 `maps/xzy_lab.yaml` 改成 `image: xzy_lab.pgm`
（相对路径按 yaml 所在目录解析，容器内外都能用）。
注意：**在 ROS1 里重存一次图会把它写回绝对路径**，跨端用之前先检查一眼：

```bash
grep '^image:' maps/xzy_lab.yaml      # 期望看到 image: xzy_lab.pgm
```

**2）初始位姿不一样。** ROS1/gmapping 的 map 原点 ≈ odom 原点，建图起点在
map(-2.0, -0.5)；Cartographer 的 map 原点是建图起点，也就是 map(0,0)。
所以用老图时初始位姿要给 (-2.0, -0.5)，不是默认的 0：

```bash
# 终端 1：仿真（车停在 spawn 点 -2.0, -0.5）
ros2 launch /workspace/launch/ros2/sim_xzy_lab.launch.py

# 终端 2：用 ROS1 的老图导航（注意 initial_pose 用老图坐标系的值）
ros2 launch /workspace/launch/ros2/navigation.launch.py \
    map_file:=/workspace/maps/xzy_lab.yaml \
    initial_pose_x:=-2.0 initial_pose_y:=-0.5 initial_pose_a:=0.0
```

两条路线随你选：**借老图**（省一次建图，5 分钟就能验证导航）或**自己建新图**
（走一遍 ROS2 建图流程，存成 `maps/ros2_xzy_lab.*`，此时初始位姿用默认的 0）。
两张图可以都留着做对照。

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
7. **`map_server` 是 lifecycle 节点**（ROS2 特有）：单跑
   `ros2 run nav2_map_server map_server ...` 只是 "Creating"，不会发 `/map`，
   要再 `ros2 lifecycle set /map_server configure` + `activate` 才开始发布。
   走 `nav2_bringup`（也就是本目录的 `navigation.launch.py`）时由 lifecycle_manager
   自动完成，不用手动管。
8. **`nav2_bringup` 不会帮你起 RViz**：它的 `bringup_launch.py` 里既没有 `use_rviz`
   参数、也没有 rviz2 节点（实测：声明的参数只有 autostart/map/params_file/slam/
   use_composition/use_sim_time 等，grep rviz 零命中），传 `use_rviz:=true` 会被静默忽略。
   所以本目录的 `navigation.launch.py` **自己起 rviz2**（配置用 TurtleBot3 的
   `tb3_navigation2.rviz`，与 ROS1 那边 `navigation.launch` 用的 RViz 配置对应）。
   想要 RViz 又看不到窗口时，先确认 `open_rviz` 没被设成 false、容器有 `DISPLAY`。

## 验证状态（重要）

- `sim_xzy_lab.launch.py`：**已在容器里实测通过**——自建世界加载、车生成在 (-2.0, -0.5)、
  `/scan` 4.98 Hz、`/odom` `/tf` `/cmd_vel` `/clock` 话题齐全、日志无报错。
- `mapping.launch.py` / `navigation.launch.py`：只做了 `--show-args` 静态加载验证
  （参数能解析、默认路径正确），**没有跑完整流程**。建图与导航的底层命令在官方
  `turtlebot3_cartographer` / `nav2_bringup` 上实测过（见 `docs/` 与本文档的坑清单），
  但这一组文件本身还没端到端验证。
