# ROS 项目测试指令清单

> 用途：改完代码、换完环境、重装系统之后，按这份清单从"静态自检"到"运行时自检"
> 逐项确认。每步都写了**期望结果**，对不上就去第 7 节的故障对照表。

## 0. 通用前提（每个新终端都要）

```bash
source /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

## 1. 环境自检（不启动任何节点，10 秒）

```bash
rosversion -d                                   # 期望：noetic
rospack find turtlebot3_gazebo turtlebot3_slam turtlebot3_navigation gmapping map_server
                                                # 期望：逐个打印路径，无 not found
python3 -c "import numpy, matplotlib; print('python deps ok')"
ls /home/xzy/xzy-project/worlds/xzy_lab.world   # 期望：文件存在
grep '^image:' /home/xzy/xzy-project/maps/map.yaml   # 期望：image: map.pgm（相对路径）
```

## 2. 官方世界：建图闭环（三个终端）

终端 1（仿真）：

```bash
roslaunch ~/xzy-project/launch/simulation_world.launch
```

终端 2（建图，自动弹 RViz）：

```bash
roslaunch ~/xzy-project/launch/mapping.launch
```

终端 3（键盘遥控）：

```bash
roslaunch ~/xzy-project/launch/teleop_keyboard.launch
```

另开终端做验证：

```bash
rostopic hz /scan                    # 期望：有稳定频率输出
rostopic hz /odom                    # 期望：有稳定频率输出
rostopic echo -n1 /scan | head -12   # 期望：ranges 里有有限值（不是全 inf）
rosnode list | grep -i gmapping      # 期望：turtlebot3_slam_gmapping
```

RViz 里确认：栅格地图随车移动持续"生长"、无重影。

绕完一圈后（新终端）存图：

```bash
roslaunch ~/xzy-project/launch/save_map.launch
ls -la ~/xzy-project/maps/map.pgm ~/xzy-project/maps/map.yaml   # 期望：两个文件都在且时间戳更新
```

## 3. 导航闭环（终端 1 保持 Gazebo）

```bash
roslaunch ~/xzy-project/launch/navigation.launch map_file:=$HOME/xzy-project/maps/map.yaml
```

RViz 里两步：`2D Pose Estimate`（设在车当前位置、拖出朝向）→ `2D Nav Goal`（点目标、拖朝向）。

运行时验证：

```bash
rostopic echo -n1 /amcl_pose                                # 期望：map 坐标系下的位姿
timeout 5 rostopic echo /tf | grep -m1 'frame_id: "map"'    # 期望：能抓到 map→odom
rostopic echo -n1 /move_base/status                         # 期望：发目标后出现 goal 状态
rosrun tf tf_monitor map odom                               # 期望：Chain is: map -> odom
```

RViz 现象：粒子云收敛成一团 → 出现绿色全局路径 → 小车沿路走、遇障绕开 → 到达自动停。

## 4. 自建世界 worlds/xzy_lab.world 测试

语法检查：

```bash
gz sdf -k /home/xzy/xzy-project/worlds/xzy_lab.world     # 期望：无 parse error
```

无头加载（不开界面，适合快速自检）：

```bash
roslaunch ~/xzy-project/launch/simulation_world.launch \
    world_file:=$HOME/xzy-project/worlds/xzy_lab.world gui:=false
```

另开终端验证：

```bash
rostopic echo -n1 /gazebo/model_states | grep '^  - '
# 期望：列出 wall_ / divider_ / cabinet_ / crate_ / pillar_ / bin_ 与机器人

python3 -c "import math,rospy; from sensor_msgs.msg import LaserScan; rospy.init_node('scan_check',anonymous=True); m=rospy.wait_for_message('/scan',LaserScan,timeout=8); r=[x for x in m.ranges if math.isfinite(x) and x < m.range_max-1e-6]; print('总束数=%d 有效命中=%d 最近=%.2f 最远=%.2f'%(len(m.ranges),len(r),min(r) if r else -1,max(r) if r else -1))"
```

期望：**有效命中占多数**（本项目骨架实测：360 束里 285 束有命中，最近 0.76 m、最远 3.50 m）。
如果命中很少（大多数 inf），说明障碍离出生点太远——激光只有 3.5 m 量程，回波全空
会让 SLAM 信息匮乏（见 `m1_slam_sim/EXPERIMENTS.md` 的退化环境对照）。

## 5. m1 离线算法与对比实验

不需要 ROS 的部分：

```bash
cd ~/xzy-project/m1_slam_sim
python3 run.py               # 期望：生成 output/m1_slam_compare.png（重影 vs 清晰）
python3 experiment.py        # 期望：更新 output/m1_param_sweep.csv 与 .png
```

需要 ROS 的同输入对比（约 2~3 分钟）：

```bash
source /opt/ros/noetic/setup.bash
python3 compare_gmapping.py --with-gmapping
# 期望：m1 RMSE ≈0.09~0.21 m；容差 IoU：m1 ≈0.39、gmapping ≈0.75；
#       生成 output/m1_vs_gmapping.png，并在 output/comparison_metrics.csv 追加一行
```

复现"退化环境发散"对照（无墙 + 前装激光）：

```bash
WORLD_WALLS=0 LASER_OFFSET=0.16 SEED=7 python3 compare_gmapping.py --with-gmapping
# 期望：gmapping 地图发散（占据格散布约 27 m）——环境信息量不足的已知现象
```

## 6. 诊断命令速查

| 命令 | 用途 |
|---|---|
| `rostopic list` / `rosnode list` | 看有哪些话题/节点，确认启动是否完整 |
| `rostopic hz /scan` | 传感器是否有数据、频率是否正常 |
| `rostopic echo -n1 /scan` | 看一帧激光的具体数值（是否全 inf） |
| `rostopic echo /tf \| grep -m1 'frame_id: "map"'` | 确认 map→odom 是否发布（**别用 tf1 的 tf_echo 下结论**） |
| `rosrun tf tf_monitor map odom` | 看 map→odom 链路与发布频率 |
| `rostopic echo -n1 /amcl_pose` | AMCL 定位结果与协方差 |
| `rostopic echo -n1 /move_base/status` | 导航目标的执行状态 |
| `rosbag info <bag>` | 查看录制数据的话题、消息数、时长 |

## 7. 故障 → 处置对照表

| 现象 | 根因 | 处置 |
|---|---|---|
| 导航启动后报 `target frame map does not exist` | 地图没加载成功（map_server 崩了） | 看 map_server 报错；确认 `map.yaml` 的 `image:` 指向实际 pgm（本项目用相对路径 `image: map.pgm`） |
| 上一条排查时 `tf_echo` 报"map 不存在" | tf1 监听器在部分环境下的假象 | 改用 `rostopic echo /tf` 直接看有没有 `frame_id: "map"`；实测 AMCL 会自动初始化并发布 map→odom |
| RViz 里车画在地图原点、激光与墙对不上 | AMCL 按默认 (0,0,0) 初始化，位姿是错的 | `2D Pose Estimate` 设在**车当前实际位置**，拖出朝向，直到激光轮廓与墙线重合 |
| gmapping 日志 `MessageFilter Dropped 100%` | 静态变换没走 `/tf_static`，或回放竞态 | 用本仓库 `make_ros_bag.py`（已做 /tf_static + 每帧携带 + 扫描延迟 0.2 s） |
| 激光几乎全是 inf | 障碍离车太远（超过 3.5 m 量程） | 缩小房间或把障碍挪近出生点（参考 `worlds/xzy_lab.world` 的设计说明） |
| 遥控松键后车不停 | `/cmd_vel` 上仍留着最后一条速度指令 | 按 `s` 发零速度；退出遥控前先按 `s` |
| 重装系统后 `rospack find` 报 not found | ROS 包没装齐 | 按 `docs/ENV_RESTORE.md` + `~/env_backup/` 快照恢复 |

## 8. 一键静态自检（可直接粘贴）

```bash
source /opt/ros/noetic/setup.bash
P=/home/xzy/xzy-project
echo "== ROS 版本 =="; rosversion -d
echo "== 关键包 =="; for p in turtlebot3_gazebo turtlebot3_slam turtlebot3_navigation gmapping map_server; do
  printf '%-24s' "$p"; rospack find $p >/dev/null 2>&1 && echo OK || echo 缺失; done
echo "== Python 依赖 =="; python3 -c "import numpy, matplotlib; print('numpy/matplotlib OK')"
echo "== launch 文件 =="; for f in simulation_world mapping teleop_keyboard navigation save_map; do
  [ -f "$P/launch/$f.launch" ] && echo "  $f.launch OK" || echo "  $f.launch 缺失"; done
echo "== 自建世界 =="; [ -f "$P/worlds/xzy_lab.world" ] && echo "  xzy_lab.world OK" || echo 缺失
echo "== 地图路径 =="; grep '^image:' "$P/maps/map.yaml"
echo "== m1 输出 =="; ls "$P/m1_slam_sim/output" | head -8
```

## 9. 运行时自检（栈已启动时粘贴）

```bash
source /opt/ros/noetic/setup.bash
echo "== 节点 =="; rosnode list | head -10
echo "== /scan 频率 =="; timeout 3 rostopic hz /scan 2>/dev/null | head -3
echo "== map→odom =="; timeout 5 rostopic echo /tf 2>/dev/null | grep -m1 -A1 'frame_id: "map"' || echo "未发现 map 帧"
echo "== AMCL =="; timeout 4 rostopic echo -n1 /amcl_pose 2>/dev/null | head -8
```
