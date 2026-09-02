#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 M1 仿真数据导出成 ROS bag（/scan + /odom + /tf + /clock）。

用途：给 gmapping 喂"和 M1 完全相同的激光/里程计"，做同输入对比。

用法（需要已 source /opt/ros/noetic/setup.bash）：
    cd m1_slam_sim && python3 make_ros_bag.py

产出：output/m1_synthetic.bag

可复现与实验控制（环境变量，均有默认值）：
    SEED          随机种子（默认 7；固定后同参数可逐位复现）
    SPEED_SCALE   里程计距离放大系数（默认 1.02）
    YAW_BIAS      里程计角速度偏置 rad/s（默认 0.0012）
    ODOM_NOISE    里程计角噪声 std（默认 0.003）
    RAY_NOISE     激光测距噪声 std（默认 0.01）
    LASER_OFFSET  激光相对 base_footprint 的前向偏移（默认 0.16；与射线
                  从车中心发出的约定一致时置 0）
    BAG_PATH      bag 输出路径（默认 output/m1_synthetic.bag）
"""

import os
import time

import numpy as np

import rosbag
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from run import next_waypoint
from world import World


def yaw_to_quat(yaw):
    return Quaternion(0.0, 0.0, float(np.sin(yaw / 2.0)),
                      float(np.cos(yaw / 2.0)))


def main():
    os.makedirs("output", exist_ok=True)
    t0 = time.time() if os.environ.get("WALL_TIME") else 0.0
    world = World(size=14.0)
    waypoints = np.array([[2.0, 2.0], [12.0, 2.0], [12.0, 12.0],
                          [2.0, 12.0]], dtype=float)

    v = 1.2
    max_r = 0.7
    seed = int(os.environ.get("SEED", "7"))
    speed_scale = float(os.environ.get("SPEED_SCALE", "1.02"))
    yaw_bias = float(os.environ.get("YAW_BIAS", "0.0012"))
    odom_noise = float(os.environ.get("ODOM_NOISE", "0.003"))
    ray_noise = float(os.environ.get("RAY_NOISE", "0.01"))
    laser_offset = float(os.environ.get("LASER_OFFSET", "0.16"))
    lidar_angles = np.radians(np.linspace(-180.0, 180.0, 181))
    max_range = 8.0
    dt = 0.1
    duration = 45.0
    n = int(duration / dt)
    scan_interval = 5
    scan_delay = 0.2
    np.random.seed(seed)

    px, py, pyaw = 2.0, 2.0, 0.0
    ox, oy, oyaw = 2.0, 2.0, 0.0
    wp_idx = 0

    bag_path = os.environ.get("BAG_PATH", "output/m1_synthetic.bag")

    # 静态变换：base_footprint -> laser（机器人固定安装）
    # 必须发到 /tf_static：tf1/tf2 都把它当"任意时刻有效"；
    # 若发在 /tf 且只写一次，tf1 只在那个时间戳有效，gmapping 的
    # MessageFilter 按扫描时间戳查询会全部外推失败（Dropped 100%）。
    ltf = TransformStamped()
    ltf.header = Header(stamp=rospy.Time.from_sec(t0 + 0.1),
                        frame_id="base_footprint")
    ltf.child_frame_id = "laser"
    ltf.transform.translation.x = laser_offset
    ltf.transform.translation.z = 0.10
    ltf.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)

    with rosbag.Bag(bag_path, "w") as bag:
        static_tf = TFMessage()
        static_tf.transforms.append(ltf)
        bag.write("/tf_static", static_tf, t=rospy.Time.from_sec(t0 + 0.1))

        # 从 t=0.1 开始，避免 t=0 的无效时间戳让 rosbag/gmapping 拒收
        for i in range(1, n):
            t = i * dt
            stamp = rospy.Time.from_sec(t0 + t)
            bag.write("/clock", Clock(clock=stamp), t=stamp)

            wp_idx = next_waypoint(np.array([px, py]), waypoints, wp_idx)
            target = waypoints[wp_idx]
            des_hdg = np.arctan2(target[1] - py, target[0] - px)
            err = (des_hdg - pyaw + np.pi) % (2 * np.pi) - np.pi
            r_cmd = float(np.clip(1.2 * err, -max_r, max_r))

            pyaw += r_cmd * dt
            px += v * np.cos(pyaw) * dt
            py += v * np.sin(pyaw) * dt

            dist = v * dt * speed_scale
            oyaw += (r_cmd + yaw_bias) * dt + np.random.normal(0, odom_noise)
            ox += dist * np.cos(oyaw)
            oy += dist * np.sin(oyaw)

            # odom -> base_footprint
            odom = Odometry()
            odom.header = Header(stamp=stamp, frame_id="odom")
            odom.child_frame_id = "base_footprint"
            odom.pose.pose.position.x = ox
            odom.pose.pose.position.y = oy
            odom.pose.pose.orientation = yaw_to_quat(oyaw)
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = r_cmd
            bag.write("/odom", odom, t=stamp)

            tf_msg = TFMessage()
            odom_tf = TransformStamped()
            odom_tf.header = Header(stamp=stamp, frame_id="odom")
            odom_tf.child_frame_id = "base_footprint"
            odom_tf.transform.translation.x = ox
            odom_tf.transform.translation.y = oy
            odom_tf.transform.rotation = yaw_to_quat(oyaw)
            tf_msg.transforms.append(odom_tf)
            # 静态变换每帧也带一份（防御：万一 /tf_static 单条消息漏接，
            # tf1 仍能在每帧时间戳上解析 laser 帧）
            laser_tf = TransformStamped()
            laser_tf.header = Header(stamp=stamp, frame_id="base_footprint")
            laser_tf.child_frame_id = "laser"
            laser_tf.transform.translation.x = 0.16
            laser_tf.transform.translation.z = 0.10
            laser_tf.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
            tf_msg.transforms.append(laser_tf)
            bag.write("/tf", tf_msg, t=stamp)

            # 真值位姿（分析用，不喂给 gmapping）
            gt = PoseStamped()
            gt.header = Header(stamp=stamp, frame_id="world")
            gt.pose.position.x = px
            gt.pose.position.y = py
            gt.pose.orientation = yaw_to_quat(pyaw)
            bag.write("/ground_truth", gt, t=stamp)

            if i % scan_interval == 0:
                ranges = [world.raycast(px, py, pyaw + a, max_range) +
                          np.random.normal(0, ray_noise)
                          for a in lidar_angles]
                scan = LaserScan()
                scan.header = Header(stamp=stamp, frame_id="laser")
                scan.angle_min = float(lidar_angles[0])
                scan.angle_max = float(lidar_angles[-1])
                scan.angle_increment = float(lidar_angles[1] - lidar_angles[0])
                scan.time_increment = 0.0
                scan.scan_time = 0.5
                scan.range_min = 0.1
                scan.range_max = max_range
                scan.ranges = [float(min(r, max_range)) for r in ranges]
                bag.write("/scan", scan, t=stamp + rospy.Duration(scan_delay))


    print("已生成 %s（时间戳起点 %.1fs）" % (bag_path, t0))
if __name__ == "__main__":
    main()
