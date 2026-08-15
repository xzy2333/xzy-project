#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1：手写 2D 栅格 SLAM 原理演示。

小车在世界里绕圈，带误差的里程计 + 激光雷达 → 占位栅格地图。
对比两组结果：
  (1) 只用里程计建图 → 漂移导致重影
  (2) 用"相关扫描匹配"修正位姿后建图 → 地图干净

这就是 SLAM 的最核心直觉：地图和位姿互相修正。
产出：output/m1_slam_compare.png + 终端打印轨迹误差对比
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Micro Hei",
                                   "SimHei", "Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

from slam import GridMap, scan_match, score_pose_img
from world import World


def next_waypoint(pos, waypoints, idx, radius=1.0):
    if np.hypot(*(waypoints[idx % len(waypoints)] - pos)) < radius:
        idx += 1
    return idx % len(waypoints)


def draw_obstacles(ax, world):
    for o in world.obstacles:
        if o[0] == "circle":
            _, cx, cy, r = o
            ax.add_patch(plt.Circle((cx, cy), r, color="0.55", alpha=0.8))
        else:
            _, cx, cy, w, h, yaw = o
            c, s = np.cos(yaw), np.sin(yaw)
            corners = np.array([[-w / 2, -h / 2], [w / 2, -h / 2],
                                [w / 2, h / 2], [-w / 2, h / 2]])
            rot = np.array([[c, -s], [s, c]])
            pts = corners @ rot.T + np.array([cx, cy])
            ax.add_patch(plt.Polygon(pts, color="0.55", alpha=0.8))


def main():
    np.random.seed(7)
    os.makedirs("output", exist_ok=True)

    world = World(size=14.0)
    waypoints = np.array([[2.0, 2.0], [12.0, 2.0], [12.0, 12.0],
                          [2.0, 12.0]], dtype=float)

    # 真值位姿
    px, py, pyaw = 2.0, 2.0, 0.0
    # 里程计位姿（会漂移）
    ox, oy, oyaw = 2.0, 2.0, 0.0
    # 扫描匹配修正后的位姿
    corr_x, corr_y, corr_yaw = 2.0, 2.0, 0.0

    v = 1.2
    max_r = 0.7
    speed_scale = 1.02      # 里程计距离放大 2%（误差来源 1）
    yaw_bias = 0.0012       # 陀螺偏置 rad/s（误差来源 2）
    lidar_angles = np.radians(np.linspace(-180.0, 180.0, 181))
    max_range = 8.0

    map_odom = GridMap(size=14.0, res=0.1)
    map_corr = GridMap(size=14.0, res=0.1)

    dt = 0.1
    n = int(45.0 / dt)
    scan_interval = 5        # 每 0.5s 一帧激光

    wp_idx = 0
    true_traj, odom_traj, corr_traj = [], [], []
    err_odom, err_corr = [], []
    scan_no = 0
    warmup_scans = 3    # 前 1.5s 直接用里程计建图，攒出初始地图再开匹配
    prev_odom = (ox, oy, oyaw)   # 上一帧里程计，用于计算增量

    for i in range(n):
        # ---- 控制：纯追踪到当前航点（只用于产生运动） ----
        wp_idx = next_waypoint(np.array([px, py]), waypoints, wp_idx)
        target = waypoints[wp_idx]
        des_hdg = np.arctan2(target[1] - py, target[0] - px)
        err = (des_hdg - pyaw + np.pi) % (2 * np.pi) - np.pi
        r_cmd = float(np.clip(1.2 * err, -max_r, max_r))

        # ---- 真值运动 ----
        pyaw += r_cmd * dt
        px += v * np.cos(pyaw) * dt
        py += v * np.sin(pyaw) * dt

        # ---- 里程计（带误差） ----
        dist = v * dt * speed_scale
        oyaw += (r_cmd + yaw_bias) * dt + np.random.normal(0, 0.003)
        ox += dist * np.cos(oyaw)
        oy += dist * np.sin(oyaw)

        # ---- 激光 + SLAM 更新 ----
        if i % scan_interval == 0:
            ranges = [world.raycast(px, py, pyaw + a, max_range) +
                      np.random.normal(0, 0.01) for a in lidar_angles]
            scan = list(zip(ranges, lidar_angles))

            if scan_no < warmup_scans:
                corr_x, corr_y, corr_yaw = ox, oy, oyaw
            else:
                # 预测：上一帧修正位姿 + 本帧里程计增量（标准做法）
                delta = np.array([ox - prev_odom[0], oy - prev_odom[1]])
                pred_yaw = (corr_yaw + (oyaw - prev_odom[2]) + np.pi) % \
                    (2 * np.pi) - np.pi
                pred = (corr_x + delta[0], corr_y + delta[1], pred_yaw)
                corr_x, corr_y, corr_yaw = scan_match(
                    map_corr, scan, pred, max_range)
                # 验收门槛：匹配分数必须明显优于预测才接受
                img = map_corr.blurred_image(passes=1)
                s_pred = score_pose_img(img, scan, pred, max_range, map_corr.res)
                s_corr = score_pose_img(img, scan, (corr_x, corr_y, corr_yaw),
                                        max_range, map_corr.res)
                if s_corr < s_pred + 0.2:
                    corr_x, corr_y, corr_yaw = pred
            prev_odom = (ox, oy, oyaw)

            map_odom.update_scan(ox, oy, oyaw, scan, max_range)
            map_corr.update_scan(corr_x, corr_y, corr_yaw, scan, max_range)

            true_traj.append((px, py))
            odom_traj.append((ox, oy))
            corr_traj.append((corr_x, corr_y))
            err_odom.append(float(np.hypot(ox - px, oy - py)))
            err_corr.append(float(np.hypot(corr_x - px, corr_y - py)))
            scan_no += 1

    rmse_odom = float(np.sqrt(np.mean(np.square(err_odom))))
    rmse_corr = float(np.sqrt(np.mean(np.square(err_corr))))

    print("== M1 SLAM 结果 ==")
    print("只用里程计的轨迹误差 RMSE: %.3f m" % rmse_odom)
    print("扫描匹配修正后的轨迹误差 RMSE: %.3f m" % rmse_corr)
    print("地图重影对比见 output/m1_slam_compare.png")

    true_traj = np.array(true_traj)
    odom_traj = np.array(odom_traj)
    corr_traj = np.array(corr_traj)

    fig, axes = plt.subplots(2, 2, figsize=(13, 12))

    # 左上：轨迹对比
    ax = axes[0, 0]
    draw_obstacles(ax, world)
    ax.plot(true_traj[:, 0], true_traj[:, 1], "k-", lw=2, label="真值轨迹")
    ax.plot(odom_traj[:, 0], odom_traj[:, 1], "r--", lw=1.2,
            label="里程计（漂移）")
    ax.plot(corr_traj[:, 0], corr_traj[:, 1], "b-", lw=1.5,
            label="扫描匹配修正")
    ax.set_xlim(0, world.size)
    ax.set_ylim(0, world.size)
    ax.set_aspect("equal")
    ax.set_title("轨迹对比（修正后 RMSE 应远小于里程计）")
    ax.legend(fontsize=8)

    # 右上：只用里程计建图
    ax = axes[0, 1]
    ax.imshow(1.0 / (1.0 + np.exp(-map_odom.logodds)), cmap="gray_r",
              origin="lower", extent=[0, world.size, 0, world.size],
              vmin=0, vmax=1)
    ax.set_title("只用里程计建图（重影/漂移）")

    # 左下：扫描匹配修正后建图
    ax = axes[1, 0]
    ax.imshow(1.0 / (1.0 + np.exp(-map_corr.logodds)), cmap="gray_r",
              origin="lower", extent=[0, world.size, 0, world.size],
              vmin=0, vmax=1)
    ax.set_title("扫描匹配修正后建图（更清晰）")

    # 右下：误差随时间
    ax = axes[1, 1]
    t = np.arange(len(err_odom)) * 0.5
    ax.plot(t, err_odom, "r--", label="里程计误差")
    ax.plot(t, err_corr, "b-", label="修正后误差")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("位置误差 (m)")
    ax.set_title("误差对比（RMSE: 里程计 %.2fm vs 修正 %.2fm）" %
                 (rmse_odom, rmse_corr))
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("output/m1_slam_compare.png", dpi=150)
    plt.close(fig)
    print("已保存 output/m1_slam_compare.png")


if __name__ == "__main__":
    main()
