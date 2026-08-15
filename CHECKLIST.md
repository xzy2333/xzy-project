# 技术栈清单（对照闫金金老师方向）

> 用途：把"要找齐的技术栈"变成可勾选的清单。每完成一项就勾掉，复试前全部打勾。
> 创建日期：2026-08-09

## 一、环境方案（Ubuntu 20.04）

本机现状（已检测）：

- [x] Ubuntu 20.04.6 LTS
- [x] Python 3.8.10 + numpy + matplotlib（可直接跑本仓库 M2/M3/M4）
- [x] git
- [x] ROS1 Noetic（desktop）+ Gazebo 11.15（已装好）
- [x] turtlebot3 + gmapping + navigation（已安装，2026-08-09 装好，见 ENV_SETUP.md）
- [ ] Docker（未安装，需要装，见下面说明）
- [ ] NVIDIA 显卡驱动（无 GPU → 深度学习模块 M6 去 AutoDL 云端租卡）

ROS 选型（详见 ENV_SETUP.md）：

- [x] **主线：ROS1 Noetic（本机已装）**。20.04 原生支持、水下生态（UUV Simulator 等）最多、中文教程最全。先把它跑通，不花一分钱。
- [ ] **备用：ROS2 Humble（Docker）**。集群/多机、Nav2、slam_toolbox、Stonefish 是 ROS2 主场，等主线跑通后再容器化补上（`docker/run.sh` 已写好）。
- [ ] 导师环境确认（公众号「先进智能导航实验室」/ B 站「AI Navi Lab」/ 问师兄），记录到 ENV_SETUP.md 第 3 节

## 一·五、主线逻辑：先车后船

自主无人系统 = **感知（SLAM 建图）→ 规划（路径）→ 控制（跟踪）→ 协同（集群）**。
算法与载体无关，变的只是模型、动力学、传感器：

| 层 | 车上（先做） | 船/UUV 上（后迁移） |
|---|---|---|
| 模型 | TurtleBot3 / 自写 URDF | USV/UUV URDF + Stonefish |
| 里程计 | Gazebo 差速轮 | DVL + IMU |
| 感知 | 2D 激光 SLAM | 声呐 SLAM（原理相同） |
| 规划 | Nav2 / A*（length 模式） | 洋流约束 A*（time 模式） |
| 协同 | 多车命名空间 | 多艇一致性/编队 |

面试话术："我本科方向是无人系统，先在地面小车把建图、导航、编队验证清楚，后续想拓展到水面/水下无人系统，所以研究了您这边的课题。"

## 二、模块清单

### M1 栅格 SLAM 原理（Python，先懂原理，10 月中旬前完成）

- [ ] 跑通 `m1_slam_sim/run.py`，看懂两张地图的区别（里程计建图重影 vs 扫描匹配修正后清晰）
- [ ] 改 `run.py` 里的 `speed_scale` / `yaw_bias`，观察漂移变大后修正是否还能跟上
- [ ] 改 `slam.py` 里 `scan_match` 的搜索窗口，观察"窗口太小跟丢 / 太大变慢"
- [ ] 写一篇博客：占位栅格地图 + 相关扫描匹配 = SLAM 的最小闭环

自测题：
1. 为什么只用里程计建图会重影？扫描匹配修正的到底是什么？
2. 栅格地图为什么用 log-odds 更新而不是直接用概率？
3. 相关扫描匹配的搜索窗口大小由什么决定？

### M2 ROS2 基础（10 月中旬前完成）

- [ ] 装好 Docker，拉取/构建 `uuv_ros2_humble` 镜像并跑通 `./run.sh bash`
- [ ] 按 `ros2_car/README.md` 跑通 turtlebot3：Gazebo 小车动起来 + Cartographer 建图 + RViz 实时显示
- [ ] 容器里跑通 ROS2 官方示例：talker/listener、turtlesim
- [ ] 理解 topic / service / action 的区别
- [ ] 会用 launch 文件启动多节点
- [ ] 会用 tf2（发布/监听坐标变换）和 rviz2（可视化）
- [ ] 能写一个简单 URDF 小车并用 rviz2 显示

自测题：
1. topic 和 service 的区别是什么？什么场景用 action？
2. 不用查资料，写出 launch 文件里启动一个节点的基本写法。
3. 说出 tf2 里 static transform 和动态 transform 的区别。

参考：鱼香ROS（fishros.com 一键安装/教程）、古月居、docs.ros.org/en/humble

### ROS1 主线（已完成，2026-08-09）

- [x] Gazebo 仿真小车动起来（turtlebot3_world）
- [x] Gmapping 建图 + map_saver 保存地图（`uuv_navi/maps/`）
- [x] AMCL 定位 + move_base 导航跑通（初始位姿 → 目标点 → 自动到达）
- [x] 博客初稿（`blog/01_gazebo_slam_navigation.md`）
- [ ] 演示视频录制（见 `blog/VIDEO_CHECKLIST.md`）

踩坑记录：仿真松键车不停（cmd_vel 无零速度）、map.yaml 的 image 路径必须随 pgm 一起搬、
初始位姿要设在建图出发点（spawn 在 (-2,-0.5)，AMCL 默认却是 (0,0,0)）。

### M3 航点控制（10 月中旬前完成）

- [ ] 跑通 `m2_waypoint_control/run.py`，看懂输出图
- [ ] 改 LOS 前瞻距离 lookahead，观察轨迹变化并记录
- [ ] 改 PID 增益，观察超调/振荡变化
- [ ] 把洋流场 current_field 改成逆流，观察轨迹是否被冲偏
- [ ] 写一篇博客：LOS 制导原理 + PID 调参经验

自测题：
1. LOS 前瞻距离越大，路径越"绕"还是越"直"？为什么？
2. PID 的 P/I/D 三项分别解决什么问题？为什么积分要抗饱和？
3. 如果洋流速度大于最大航速，会发生什么？怎么处理？

### M4 洋流路径规划（10 月中旬前完成）

- [ ] 跑通 `m3_path_planning/run.py`，理解"最短路径"和"最短时间路径"的区别
- [ ] 看懂 `edge_time` 为什么用 `L / (V + C·e)` 作为时间成本
- [ ] 改洋流场/障碍物重新出图，记录对比数据
- [ ] 写一篇博客：A* 在洋流下的成本函数设计 + 启发函数可采纳性

注意：在车上先做 `length` 模式（等价 Nav2 全局规划器干的活）；
`time` 模式（考虑洋流）是后面迁移到船时才加上的，正好是闫老师论文的核心。

自测题：
1. 为什么"长度最短"不等于"时间最短"？举例说明。
2. A* 的时间启发函数为什么要除以 `V + 最大流速`？
3. 逆流走和顺流走，时间成本函数里会发生什么？

### M5 集群编队（考完研后完成）

- [ ] 跑通 `m4_swarm/run.py`，观察编队误差是否收敛
- [ ] 看懂领航-跟随 + 势场避碰的实现
- [ ] 把领航路径换成 M3 规划的路径，做"规划→编队跟踪"闭环
- [ ] 读 Olfati-Saber 一致性论文（摘要+结论部分即可）
- [ ] 写一篇博客：从单艇到集群，多出的难点是什么

自测题：
1. 为什么要加"一致性"？只跟踪编队目标位置有什么问题？
2. 势场法为什么可能陷入局部极小值？
3. 水下集群比地面集群多出哪些约束（通信、洋流）？

### M6 定位 / SLAM（考完研后完成）

- [ ] 手写 EKF：GPS+IMU 松耦合融合（Python），画轨迹对比真值、算 RMSE
- [ ] 在 Docker 容器里跑通 Cartographer 单艇建图
- [ ] 写 3000 字集群 SLAM 调研：方法分类、代表工作、难点

自测题：
1. EKF 的预测步和更新步分别对应什么物理意义？
2. 集群 SLAM 相比单艇 SLAM 多出哪三类问题（地图、位姿、通信）？

参考：高翔《视觉SLAM十四讲》、《概率机器人》

### M7 水下目标识别（可选，考完后/云端）

- [ ] AutoDL 租一张显卡，YOLOv8n 微调公开水下数据集（Brackish）
- [ ] 对比不同模型大小/推理速度，记录 mAP 和 FPS
- [ ] 封装成 ROS2 节点发布检测结果

自测题：
1. 轻量化模型（n/s/m）和精度/速度的取舍怎么量化？
2. 水下图像相比地面图像有什么特点（模糊、偏色、光照）？

## 三、复试素材检查单

- [ ] M1：SLAM 对比图（`m1_slam_sim/output/m1_slam_compare.png`）
- [ ] M2：航点跟踪轨迹图（`m2_waypoint_control/output/m2_trajectory.png`）
- [ ] M2：控制量/误差图（`m2_waypoint_control/output/m2_control.png`）
- [ ] M3：洋流 vs 无洋流路径对比图（`m3_path_planning/output/m3_compare.png`）
- [ ] M3：对比数据表（路径长度 / 航行时间）
- [ ] M4：编队仿真图（`m4_swarm/output/m4_formation.png`）
- [ ] 每个模块一篇技术博客（CSDN/知乎）
- [ ] ROS1 主线博客发布（初稿：`blog/01_gazebo_slam_navigation.md`）
- [ ] 每个模块 3~5 分钟演示视频（B 站）
- [ ] GitHub 仓库 README 画架构图

## 四、参考链接

- 闫金金老师主页：http://faculty.hrbeu.edu.cn/yanjinjin/zh_CN/index.htm
- 洋流下 AUV 路径规划论文（Drones 2024）：https://www.mdpi.com/2504-446X/8/8/348
- 集群仿真平台综述（Robotics 2023）：https://www.mdpi.com/2218-6581/12/2/57
- Stonefish 水下仿真器：https://github.com/patrykcieslak/stonefish
- 鱼香ROS：https://fishros.com
- 古月居：https://www.guyuehome.com
- ROS2 Humble 文档：https://docs.ros.org/en/humble
- 高翔《视觉SLAM十四讲》：https://github.com/gaoxiang12/slambook
