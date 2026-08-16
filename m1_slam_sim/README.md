# M1：手写 2D 栅格 SLAM 原理

在 ROS2/Gazebo 之前，先用纯 Python 把 SLAM 的**核心直觉**跑通：

1. 激光打中墙 → 栅格"占据"；射线穿过 → 栅格"空闲"（log-odds 更新）
2. 里程计会漂移 → 每帧激光在里程计位姿附近搜索一个"地图分数最高"的位姿（相关扫描匹配）
3. 地图和位姿互相修正，这就是 SLAM 的最小闭环

运行：

```bash
cd ~/xzy-project/m1_slam_sim
python3 run.py
```

产出 `output/m1_slam_compare.png`：左边轨迹对比，右边两张地图（里程计建图重影 vs 修正后清晰）。

动手实验：
- 改 `run.py` 里的 `speed_scale`、`yaw_bias`，漂移越大，重影越明显
- 改 `slam.py` 里 `scan_match` 的 `window`，窗口太小会跟丢，太大搜索慢
- 这就是 Cartographer/slam_toolbox 在 Gazebo 里做的事的"微型版"——先在脑子里有这张图，再看 ROS2 里的现成工具就不晕了

