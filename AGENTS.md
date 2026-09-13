# xzy-project —— 新助手须知

备考生 xzy 的机器人仿真项目，面向哈工程青岛基地（闫金金老师方向），路线是"先车后船"。

沟通方式：中文、口语化、便于背诵。先给结论和数字，再给依据；未验证的必须标注；
不接受"听起来合理"的机制解释。每个数字要有出处，不同口径不许混用。

文档入口：命令清单 COMMANDS.md、环境 ENV_SETUP.md、学习清单 CHECKLIST.md、
证据文档 docs/ 与 m1_slam_sim/EXPERIMENTS.md。**不要把这些内容抄进本文件。**

## 两套编号别混（最容易踩）

- `m1_slam_sim`（栅格 SLAM 演示）、`m2_waypoint_control`（LOS+PID 航点）、
  `m3_path_planning`（洋流 A*）、`m4_swarm`（领航-跟随编队）是**纯 Python 数值模块**，
  不依赖 ROS，`cd mN_xxx && python3 run.py` 直接跑，产物在各自 `output/`。
- `CHECKLIST.md` 里的 M1–M7 是**学习清单**，不是目录。
  对应：m1↔M1、m2↔M3、m3↔M4、m4↔M5；M2(ROS2)、M6(EKF/SLAM)、M7(水下识别) 暂无目录。

## 目录约定

launch 全在 `launch/`；地图 `maps/`；世界 `worlds/`；脚本 `scripts/`；
证据文档 `docs/` 与 `m1_slam_sim/EXPERIMENTS.md`；博客 `blog/`。

关键资产与默认值：

- 自建世界 `worlds/xzy_lab.world`（8×6 m，手写 SDF，每模型一个 pose）；
  自建地图 `maps/xzy_lab.*`；参数对照地图 `maps/lab_{A..E}*.{pgm,yaml}`。
- **建图默认参数 = E 组**（`launch/mapping.launch` 默认 `gmapping_params` 指向
  `launch/gmapping_params_E_fine.yaml`）；要用官方参数需显式指定。
- `launch/mapping.launch` 支持 `open_rviz:=false`；`launch/simulation_world.launch`
  支持 `world_file:=` 与 `gui:=false`；`launch/navigation.launch` 已透传
  `initial_pose_x/y/a`（默认仍是 0），并支持 `open_rviz:=false`。
- 工具脚本在 `scripts/`：`list_world_models.py`（看/改世界布局）、
  `map_quality.py`（真值栅格+IoU 评估地图）、`drive_lab.py`（固定路线）、
  `nav_goal_test.py`（批量目标点验收）、`gmapping_param_sweep.sh`（参数对照，组名用
  环境变量 `SWEEP_GROUPS` 传入）。

## 环境坑（本机 Ubuntu 20.04）

- sudo 有密码、非免密；提权走 `pkexec`（弹图形认证）。沙箱里 sudo 报错纯属假象。
- 沙箱看不到提权启动的进程。判断有无残留 Gazebo/rosmaster 必须用提权视角。
- 沙箱内 DNS 不通：ROS 走 localhost 必须非沙箱执行；外网 apt/npm 用国内镜像；
  GitHub 走 SSH，私有仓库匿名 HTTPS 一律 404。
- 同一时刻只能有一个 Gazebo（占 11345）和一个 roscore（占 11311）。
  重复启动表现为 `Spawn service failed`。
- **杀进程只用 `pkill -x` 精确匹配或直接杀 PID。`pkill -f` / `pgrep -f` 会匹配到自身命令行**
  （曾自杀两次、误杀用户会话一次）。
- 用户会自己开终端跑 Gazebo/导航：动进程前先确认归属，绝不用宽泛匹配。
- 16 核 / 15 GB 内存 / 约 513 GB 空闲；无 GPU。
- Docker 未安装。ROS2 走容器：Humble 面向 22.04，本机 20.04 原生只有 ROS1，
  直接装会与系统库/相机 SDK 纠缠。
- 坐标系事实：odom 原点 = 世界原点；车 spawn 于 (-2.0, -0.5)；建图起点 ≈ map(-2, -0.5)；
  AMCL 默认初始位姿 (0,0,0)。
- 两套激光参数**不可混用**：离线仿真 181 束 / 2° / 8 m；Gazebo 实时 360 束 / 3.5 m。
- `map_server` 的相对 `image:` 按 yaml 所在目录解析。相机 SDK 在 `/opt/MVS`（非 apt 包）。
- bash 保留变量（如 GROUPS、UID）不能当普通变量名：曾把脚本的组名解析成 1000。
- 提权包装传参/传环境变量是正常的：曾误判为包装问题，实际是脚本自身的 bug。

## 已证伪的说法（别再当成结论提出）

- "合成世界是 14 m 空房间、四面墙对称" —— 假。`m1_slam_sim/world.py` 无边界墙、仅 5 个障碍。
- "gmapping 发散源于系统性里程计漂移 / 激光 0.16 m 安装偏移" —— 假。去掉漂移、偏移归零仍发散。
- "198 m 地图证明位姿发散" —— 假。那是画布配置（±100 m）；真信号是占据格散布约 27 m。
- "手写版 m1 优于 gmapping" —— 假。带墙公平对比：gmapping 容差 IoU ≈ 0.75，m1 ≈ 0.39（5 个种子）。
- "不设初始位姿导航会卡死" —— 假。AMCL 会按 (0,0,0) 初始化并立即发布 map→odom；
  真问题是位姿与建图起点差约 2.1 m。

**gmapping 发散根因（已定论）**：无墙 + 障碍稀疏 + 量程截断 → 各位置观测几乎全同
（极端感知混叠，等价于信息量不足）→ 似然无区分度 → 粒子参数漂移。
加 4 面边界墙后，即使带漂移也收敛。

## 试过但放弃的手段（别重走）

- tf1 的 `tf_echo` 判坐标系：本机不可信（假报 "map 不存在"），改用直接读 `/tf` 或 tf2。
- FFT 互相关做地图对齐：出现伪峰（15.6 m），改为有界穷举（±4 m）。
- `icp_match`（SVD+最近邻）：未验证、演示行为不一致，已删除，不留死代码。
- 用 `rostopic echo -n1 /cmd_vel` 验证"松键不停"：读不到历史消息，改用频率判据。
- 裸 `rosrun gmapping` 建图：缺 robot_state_publisher → 100% 丢帧，必须走 `mapping.launch`。

## 参数实验结论

同一世界、同一 85 s 固定路线；IoU 为双侧容差 2 格。

| 组 | 设置 | 更新帧 | IoU | 覆盖 |
|---|---|---|---|---|
| A | 官方默认 | 169 | 0.432 | 7.5×6.0 m |
| B | 只调细距离/角度阈值 | 200 | 0.426 | 7.3×6.0 m（无收益） |
| C | 放宽匹配 minimumScore 50→30、maxUrange→3.5 | 171 | 0.452 | 8.0×6.0 m（收益最大） |
| D | C + temporalUpdate 0.5→2.0 | 73 | 0.420 | 更新真正由运动触发 |
| E | D + 细阈值（0.4 m / 0.1 rad） | 108 | 0.452 | 与 C 同质量，更新少约 1/3 |

结论：B 的无收益是 temporalUpdate 造成的"参数遮蔽"；**推荐 E**，
若需车静止也刷新地图则用 C。

m1 口径：逐格 IoU 对墙厚极敏感，结论以"双侧容差 2 格"版为准；
m1 轨迹 RMSE 单种子波动 0.09–0.21 m，只能报范围；真值仅存在于仿真（真实轨迹是程序积分出来的）。

## 验收标准（"算完成"的判据）

- m1：`run.py` 出重影/清晰对比图；`experiment.py` 出参数扫描 CSV/PNG；
  `compare_gmapping.py` 出对比图并追加指标行；所有结论标注口径。
- m2：出轨迹图与控制量图，带洋流扰动仍能跟踪收敛。
- m3：出"最短路径 vs 最短时间路径"对比图与指标。
- m4：出编队图与编队误差收敛。
- ROS1 主线：世界可加载且激光有效回波占多数；建图无重影、无丢帧；地图 yaml 用相对路径；
  导航启动即定位在建图起点、三个目标点全部到达、能绕开中途插入的未知障碍（代价地图可见致命格）。
- 文档：每个数字有出处；不同口径不混用；不写未验证的机制解释。

## 提交与汇报

- 小步提交、中文提交信息（改了什么 + 为什么），推 origin main（SSH）。
- 先结论与数字，再给依据；未验证必须标注。
- 结论要有实测支撑，别用"应该可以"；长会话上下文涨到 20 万以上时主动压缩，别等自动触发。

## 现状（2026-09-13）

m1 仍是离线 Python 工具，尚无 ROS 节点；m2–m4 未接入 ROS；ROS2/Nav2 未迁移。
仓库私有；PDF 导出物早于文档重写已被删除，需从定稿 markdown 重导。
