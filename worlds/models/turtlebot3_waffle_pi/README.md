# 精简版 waffle_pi 模型（"只要车"）

本机没有独立显卡，Gazebo 走 Mesa 软件渲染，**每一个像素都是 CPU 画的**。实测
`gzclient` 占 454% CPU、`gzserver` 占 354%（16 核机器负载 ~24，已经超额订阅），
而官方 waffle_pi 模型里带的 30 Hz 相机（640×480）实测只能跑到 13→9 Hz，说明渲染早
就饱和了——而 SLAM / 导航根本不用这个相机（用的是 `/scan`）。

所以这里放一份精简版模型，只保留"车"本身需要的东西：

| 传感器 / 设置 | 官方模型 | 本目录精简版 |
|---|---|---|
| 激光 `hls_lfcd_lds`（发 `/scan`） | 有 | **保留** |
| IMU `tb3_imu`（发 `/imu`） | 有 | **保留** |
| 差速驱动 `turtlebot3_diff_drive` | 有 | **保留** |
| 关节状态 `turtlebot3_joint_state` | 有 | **保留** |
| 相机 sensor + `libgazebo_ros_camera.so`（640×480@30Hz） | 有 | **删除** |
| 激光射线可视化（Gazebo 里的蓝线） | `<visualize>true</visualize>` | `<visualize>false</visualize>` |

保留 `camera_link` / `camera_rgb_optical_frame` 两个空的 link 和 joint：它们只是 TF 坐标
系，不产生任何渲染或计算开销，删掉反而容易和 `robot_state_publisher` 发布的 URDF 对不上。

## 怎么生成的（可复现）

```bash
# 从镜像里取出官方模型
sudo docker cp <容器>:/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi /tmp/tb3_model_orig

# ① 第 142 行：激光可视化关掉；② 删掉 385~419 行：整个相机 sensor 段
sed -e '142s|<visualize>true</visualize>|<visualize>false</visualize>|' \
    -e '385,419d' /tmp/tb3_model_orig/model.sdf > model.sdf
cp /tmp/tb3_model_orig/model.config model.config

# 校验
gz sdf -k model.sdf        # → Check complete
```

网格文件（`model://turtlebot3_common/meshes/...`）仍从系统装的 turtlebot3_gazebo 包里解析，
所以这里只需要放 `model.sdf` + `model.config`。

## 怎么用 / 怎么对比

`launch/ros2/sim_xzy_lab.launch.py` 默认就用这份精简模型；想跑官方完整版做对照，
把 `robot_sdf` 指回官方文件即可：

```bash
# 默认（精简版，只要车）
ros2 launch /workspace/launch/ros2/sim_xzy_lab.launch.py

# 对照（官方完整模型：带相机 + 激光可视化）
ros2 launch /workspace/launch/ros2/sim_xzy_lab.launch.py \
    robot_sdf:=/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf
```

对比时可以看的量（都在容器里测）：

```bash
ros2 topic hz /camera/image_raw     # 精简版：没有这个话题；完整版：实测只有 13→9 Hz
ros2 topic hz /scan                 # 两版都应该是 ~5 Hz
top -bn1 | grep -E "gzserver|gzclient"   # 看 CPU 占用变化
```

> 注：`<shadows>true</shadows>`（`worlds/xzy_lab.world` 第 36 行）也会明显吃软件渲染性能，
> 这里没动，需要的话单独改、单独对比。
