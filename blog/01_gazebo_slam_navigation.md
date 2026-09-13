# TurtleBot3 在 Gazebo 中跑通 SLAM 建图与自主导航（ROS1 Noetic 实战 + 踩坑记录）

> 项目：[xzy-project](../README.md) · 环境：Ubuntu 20.04 + ROS1 Noetic + Gazebo 11 · 车型：waffle_pi
> 状态：已跑通（2026-08-09）· 配套视频清单见 [VIDEO_CHECKLIST.md](VIDEO_CHECKLIST.md)

## 1. 背景

本项目的技术主线是"先车后船"：把自主无人系统的四层关键技术——**感知 → 规划 → 控制 → 协同**——先在地面小车（TurtleBot3 仿真）上跑通，再迁移到水面/水下无人系统。算法与载体无关，变的只是模型、动力学和传感器。

本博客记录的是这条主线上**已经完成**的第一个完整闭环：在 Gazebo 仿真里用
Gmapping 实时建图，保存地图，再加载地图用 AMCL 定位 + move_base 导航，让小车
自己规划路径走到目标点。"感知 → 定位 → 规划 → 控制"整条链路第一次串起来。
多车协同与水面迁移属于后续阶段（仓库里 m2/m3/m4 目前是纯数值仿真，尚未接入
ROS），本文只对已完成的部分下结论。

## 2. 环境

| 项 | 值 |
|---|---|
| 系统 | Ubuntu 20.04.6 LTS |
| ROS | ROS1 Noetic（desktop） |
| 仿真 | Gazebo 11.15 |
| 车型 | `waffle_pi`（360° 激光 + RGB 相机） |
| 关键包 | `turtlebot3` / `turtlebot3-simulations` / `navigation` / `gmapping` |

```bash
sudo apt update
sudo apt install -y ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations \
    ros-noetic-navigation ros-noetic-gmapping
```

## 3. 核心概念速览（一句话版）

- **SLAM（Gmapping）**：激光 + 里程计实时构建栅格地图（occupancy grid），同时修正车自身的位置。它就是"边开车边画地图"。
- **定位（AMCL）**：给车一张已知地图，用粒子滤波估计车在地图中的位置和朝向。粒子云收敛成一团 = 定位成功。
- **规划与控制（move_base）**：全局规划器找一条从当前位置到目标点的路径，局部规划器（DWA）实时避障，最终输出 `/cmd_vel` 速度指令。
- **TF 坐标系链**：`map → odom → base_footprint → base_link → base_scan`。其中 `map→odom` 由 AMCL 发布，`odom→base_footprint` 由里程计发布，`base_footprint` 往下的关节/传感器变换由 URDF 和 robot_state_publisher 发布。

补充两个常被追问的区别：`base_footprint` 是机器人在地面的投影点，里程计以它为
参考（固定在车体正下方，不随底盘颠簸旋转）；`base_link` 是车体坐标系原点
（URDF 定义）。激光等传感器的安装偏移（如 `base_link → base_scan`）都挂在
`base_link` 之下。

## 4. 建图实操

开三个终端，按顺序执行（每个新终端都要先设车型环境变量）：

> 以下指令是项目自定义 launch（封装官方 turtlebot3 包），对照表见 [launch/README.md](../launch/README.md)。

**终端 1 —— 仿真世界**

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch ~/xzy-project/launch/simulation_world.launch
```

`turtlebot3_world` 是官方室内场景：多段墙体围合的房间里散布立柱等障碍，特征
密集——这是它能顺利建图的前提（特征匮乏是 SLAM 的退化场景，见博客 02 实验三
的对照）。

**终端 2 —— SLAM 建图（自动弹出 RViz）**

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch ~/xzy-project/launch/mapping.launch
```

**终端 3 —— 键盘遥控**

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch ~/xzy-project/launch/teleop_keyboard.launch
```

`w` 前进、`a/d` 转向、`s` 停车，让车在房间里慢慢绕一圈，RViz 里的栅格地图会随激光扫描一点点"长"出来。转完保存地图：

```bash
mkdir -p ~/xzy-project/maps
roslaunch ~/xzy-project/launch/save_map.launch
```

生成 `map.pgm`（栅格图像）+ `map.yaml`（地图元数据）。

## 5. 导航实操

**终端 1 保持 Gazebo 开着**，新开终端：

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch ~/xzy-project/launch/navigation.launch map_file:=$HOME/xzy-project/maps/map.yaml
```

RViz 弹出后按顺序做两件事：

1. **设初始位姿**：点工具栏 `2D Pose Estimate`，在地图上车实际所在的位置（**建图出发点**，见坑 3）按住并拖出车头朝向，松手。
2. **给目标点**：点 `2D Nav Goal`，在地图另一头点一下，拖出目标朝向，松手。

正常现象：红色粒子云先收敛成一团 → 出现绿色全局路径 → 小车沿路径行驶、遇障碍绕开 → 到达自动停。

## 6. 踩坑记录（本篇重点）

### 坑 1：松开键盘，小车一直不停

这不是"没阻力"。Gazebo 的差速驱动插件只认 `/cmd_vel` 上最新一条速度指令，而
teleop 节点**按下键时发送速度、松开不补发零速度**（是否自动连发取决于终端按键
重复），于是小车"巡航"在最后一条指令上。部分真车（如 TurtleBot3 的 OpenCR
固件）有看门狗——几百毫秒收不到新指令自动停；仿真里没有这个机制。

**结论**：想停就按 `s`（发零速度）；退出遥控前先按 `s`，否则最后一条非零指令会一直留在话题上。

验证：遥控时按一下前进键再松键，另开终端跑 `rostopic hz /cmd_vel`——
松键后话题频率归零（不再有新指令），而 Gazebo 里小车仍在动，说明它执行的
还是最后那条非零速度。

### 坑 2：map_server 启动即崩（exit 255）

报错：

```
ERROR: failed to open image file "/home/xzy/map.pgm": Couldn't open /home/xzy/map.pgm
[map_server-2] process has died [pid ..., exit code 255]
```

原因：`map.yaml` 里的 `image:` 是**绝对路径**。把 `map.pgm`/`map.yaml` 从家目录搬进 `xzy-project/maps/` 时只搬了文件、没改 yaml 里的路径，map_server 按旧路径找不到图片，直接退出。

**结论**：`image:` 路径必须始终指向 `map.pgm` 的实际位置，两个文件必须一起搬。
Noetic 的 map_server 会把**相对路径按 map.yaml 所在目录解析**（本机实测），
所以最省事的写法是 `image: map.pgm`（两个文件同目录，搬走即用）；用绝对路径
也行，但搬家后必须同步改。搬完检查：

```bash
cat ~/xzy-project/maps/map.yaml   # 本项目应显示 image: map.pgm（同目录相对路径）
```

### 坑 3：不设初始位姿，导航"能跑"但定位是错的

很多教程说"不设初始位姿导航会卡死"，本机实测**不成立**：我们的 launch 给 AMCL
配了初始位姿参数 `(0, 0, 0)`，Noetic 的 AMCL 收到激光后会自动按该位姿初始化
并发布 `map → odom`（实测不点 `2D Pose Estimate`，`/tf` 里 `map→odom` 立即
存在，`/amcl_pose ≈ (0, 0)`）。

真正的问题在**位姿错了**：`turtlebot3_world.launch` 里车的生成点是
**(-2.0, -0.5)**（新启动时 `odom→base_footprint ≈ (-2.0,-0.5)`，即 odom 原点在
世界原点），而 AMCL 自动初始化的位姿是地图原点 **(0, 0)**——两者相差约
**2.1 m**。所以 RViz 里车被画在地图原点附近（视图上接近画面中央），激光扫出的
墙轮廓自然和地图墙线对不上。

关键认知：**RViz 里车的位置不是从 Gazebo 读来的，而是 AMCL 认为的位置。**
初始位姿给错，后面全错；而"给对"的方法不是依赖默认值，而是主动设一次。

**结论**：`2D Pose Estimate` 要设在**车当前实际所在位置**对应的地图点
（新建图后首次导航通常就是建图出发点；若车已移动过，以车当前位置为准），
箭头朝车头方向。最稳妥的办法是边拖边看激光轮廓，和地图墙线重合了就松手。

> 补充：如果启动日志出现 `target frame map does not exist`，那通常不是
> "没设初始位姿"，而是**地图根本没加载成功**（坑 2：map_server 崩了），或
> 排查工具本身的问题（本环境的 tf1 `tf_echo` 就曾假报"map 不存在"，改用
> `rostopic echo /tf` 直接看帧即可确认）。

## 7. 结果验证清单

- [x] `rostopic echo -n1 /scan` 有激光数据
- [x] tf 树存在 `map → odom → base_footprint → ...` 完整链路（`rosrun tf tf_monitor map odom`）
- [x] 粒子云从散开收敛聚成一团
- [x] 激光轮廓与地图墙线重合
- [x] 发目标后出现全局路径 → 局部路径 → 小车到达自动停

## 8. 与手写 SLAM 的联系（m1）

本仓库 m1 模块是手写的 2D 栅格 SLAM：log-odds 栅格更新 + 相关扫描匹配。
Gmapping 可以看成它的"工业版"——同样是栅格地图 + 扫描匹配，只是加了粒子滤波
和更完整的工程实现。先看懂 m1 的最小闭环，再操作 Gmapping 时概念完全对得上。
仓库里已把 m1 与 Gmapping 做成**同一份数据的同输入对比**（含退化环境对照），
方法与结论见博客 02 与 `m1_slam_sim/EXPERIMENTS.md`。

## 9. 下一步

1. ~~用 Gazebo Building Editor 搭一个自己的环境~~ → **已完成，见第 10 节**
2. 迁移到 ROS2 Nav2（集群阶段的主场）
3. 多车编队：把 m4 领航-跟随接进 Gazebo，开三台 TurtleBot3
4. 每完成一环录 30 秒视频存档（见 VIDEO_CHECKLIST.md）

## 10. 从官方 demo 到自建环境（里程碑 A）

跑通官方 `turtlebot3_world` 只能证明"会跑流程"。这一节把环境换成自己写的，
验证整条链路不依赖官方资源。

### 10.1 环境设计

- 尺寸 8 m × 6 m，四面外墙（0.15 m 厚）+ 中间一道隔断墙，隔断在 y≈0 处留了
  1.2 m 的门洞，把房间分成东西两间；
- 障碍刻意**不对称**：西间有长凳和两根立柱，东间有柜子、货箱、垃圾桶，
  避免"四面全对称"这种 SLAM 退化场景（对照见博客 02 的实验三）；
- 环境是**手写 SDF**（`worlds/xzy_lab.world`，每个障碍一个 `<model>`，
  改 `<pose>x y z roll pitch yaw</pose>` 即可移动），不依赖任何外部 mesh；
- 看/改布局：`python3 scripts/list_world_models.py`（打印所有模型的位姿与尺寸）。

### 10.2 流程（与官方案例完全一致，只多一个参数）

```bash
# 终端 1：自建世界
roslaunch ~/xzy-project/launch/simulation_world.launch \
    world_file:=$HOME/xzy-project/worlds/xzy_lab.world
# 终端 2：建图（自带 robot_state_publisher；open_rviz:=false 可无界面）
roslaunch ~/xzy-project/launch/mapping.launch
# 终端 3：遥控走遍两个房间（也可用脚本路线：python3 scripts/drive_lab.py）
roslaunch ~/xzy-project/launch/teleop_keyboard.launch
# 存图（换名字，不覆盖官方地图）
rosrun map_server map_saver -f ~/xzy-project/maps/xzy_lab
# 导航（initial_pose 直接给出，启动即在建图起点）
roslaunch ~/xzy-project/launch/navigation.launch \
    map_file:=$HOME/xzy-project/maps/xzy_lab.yaml \
    initial_pose_x:=-2.0 initial_pose_y:=-0.5 initial_pose_a:=0.0
```

### 10.3 结果（量化，不靠"看起来不错"）

| 指标 | 官方地图 `maps/map.*` | 自建地图 `maps/xzy_lab.*` |
|---|---|---|
| 地图尺寸 | 384×384 @0.05 m | 384×384 @0.05 m |
| 占据格数 | 892 | 1892 |
| 占据范围 | 5.6 × 5.2 m | **8.0 × 6.0 m**（房间真值 8.2 × 6.1 m） |
| 与真值 IoU（原始 / 容差 2 格） | —（官方世界无真值栅格） | **0.239 / 0.665** |

工具：`scripts/map_quality.py`（从 world 文件渲染真值栅格再算 IoU）。
原始 IoU 偏低是"逐格比对 + 地图墙比真值薄"造成的，这也是我们一直用"容差 2 格"
口径的原因（和博客 02 的结论一致）。**尺寸与面积完全吻合**说明几何关系是对的。

### 10.4 这一轮踩到的新坑

1. **裸 `rosrun gmapping` 会 100% 丢帧**：官方 SLAM launch 会顺带启动
   `robot_state_publisher`（激光坐标系靠它发布），自己手起 gmapping 就少了它，
   消息过滤器按扫描时间戳查不到变换。→ 本仓库封装 `mapping.launch` 解决。
2. **官方导航 launch 不透传初始位姿**：`amcl.launch` 有 `initial_pose_x/y/a`，
   但上层没往下传，导致每次启动 AMCL 都落在地图原点（看起来像"车在地图中心"）。
   → 本仓库 `navigation.launch` 重新拼装并透传这三个参数。
3. **同一台机器只能开一个 Gazebo**：另起一个 `gzserver` 会因端口冲突直接退出，
   表现为 `Spawn service failed`。启动前先确认没有别的 Gazebo 在跑。
4. **地图文件命名与相对路径**：自建地图另存为 `maps/xzy_lab.*`，`yaml` 里的
   `image:` 用相对路径（Noetic 的 map_server 按 yaml 所在目录解析）。

### 10.5 导航验收（脚本化，可复现）

用 `scripts/nav_goal_test.py` 依次发三个目标点（map 坐标），自动记录到达与耗时：

| 目标点 | 位置说明 | 结果 | 耗时 |
|---|---|---|---|
| (-1.0, 1.5) | 西侧房间北侧 | 到达 | 13.0 s |
| (2.0, 0.5) | 东侧房间（需穿过 1.2 m 门洞） | 到达 | 18.0 s |
| (-3.2, -2.2) | 西侧房间南侧 | 到达 | 33.6 s |

**绕障测试**：运行中途用 `rosrun gazebo_ros spawn_model` 往
(-2.1,-0.4)（正好在前两点连线上）插入一个 0.8 m 见方的箱子。验证两点：

1. 障碍确实被感知——全局代价地图在该箱子附近出现 **233 个致命格（cost=100）**；
2. 同一目标点仍能到达——先把车开回 (-3.2,-2.2)（24.6 s），再发 (-1.0,1.5)，
   **27.1 s 到达**（无箱子基线 27.0 s）。

绕行耗时几乎不变，是因为 move_base 的膨胀半径（1.0 m）本来就要求路径与障碍保持
距离，插入箱子后路径只是略微外移；这也解释了为什么"重规划"在这种小场景里
不易被肉眼察觉。

### 10.6 gmapping 参数对照实验（同一路线跑三组）

官方参数不是为这个 8×6 m 小房间调的，所以我做了三组对照：同一世界、同一条
85 秒固定路线（`scripts/drive_lab.py`），只改参数，比地图质量（`scripts/map_quality.py`
算与真值栅格的容差 IoU）：

| 组 | 改动 | 容差 IoU | 覆盖范围 | 占据格 |
|---|---|---|---|---|
| A 官方默认 | — | 0.432 | 7.5×6.0 m | 1012 |
| B 更细更新 | `linearUpdate 1.0→0.4`、`angularUpdate 0.2→0.1` | 0.426 | 7.3×6.0 m | 1042 |
| C 更宽容匹配 | `minimumScore 50→30`、`maxUrange 3.0→3.5` | **0.452** | **8.0×6.0 m** | 1096 |

（房间真值 8.2×6.1 m；三组都没有丢帧告警。）

两个结论：**C 组最好**——把匹配门槛从 50 降到 30、并用满激光量程，减少了"分数
不够就丢帧"，地图覆盖更完整（8.0×6.0 m 与真值吻合）；**B 组没有收益**——因为
这组参数里 `temporalUpdate（0.5 s）` 才是主导，扫描约 2 Hz，几乎每帧都会更新，
所以把线性/角更新阈值调小并不会带来更多更新。想验证"更细更新"的价值，得同时
把 `temporalUpdate` 调小再比。

参数文件在 `launch/gmapping_params_{A_official,B_fine,C_lenient}.yaml`，切换方式：
`roslaunch ~/xzy-project/launch/mapping.launch gmapping_params:=<文件>`；
完整实验记录见 `docs/gmapping_param_sweep.md`。

## 11. 参考资料

- ROBOTIS e-Manual：TurtleBot3 仿真/建图/导航
- ROS Wiki：gmapping / amcl / move_base / map_server
- [ENV_SETUP.md](../ENV_SETUP.md)（环境与主线命令）
