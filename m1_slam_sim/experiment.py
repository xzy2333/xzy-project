#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1 参数实验：漂移大小 / 搜索窗口对建图质量的影响。

用法：
    cd m1_slam_sim && python3 experiment.py

产出：
    output/m1_param_sweep.csv   每档参数的位置 RMSE、地图 IoU、耗时
    output/m1_param_sweep.png   对比图（RMSE 与地图 IoU）

对应自测题 3 的"实测答案"：窗口太小跟丢、太大变慢，
漂移越大，修正的收益越明显。
"""

import csv
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Micro Hei",
                                   "SimHei", "Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

from slam import GridMap, scan_match
from world import World
from run import next_waypoint


def world_grid(world, res=0.1):
    """把仿真世界的障碍物渲染成真值栅格（占=1，空=0），用来算地图 IoU。"""
    n = int(world.size / res)
    grid = np.zeros((n, n), dtype=bool)
    for o in world.obstacles:
        if o[0] == "circle":
            _, cx, cy, r = o
            ys, xs = np.mgrid[0:n, 0:n]
            gx, gy = (xs + 0.5) * res, (ys + 0.5) * res
            grid |= (gx - cx) ** 2 + (gy - cy) ** 2 < r * r
        else:
            _, cx, cy, w, h, yaw = o
            c, s = np.cos(yaw), np.sin(yaw)
            ys, xs = np.mgrid[0:n, 0:n]
            gx, gy = (xs + 0.5) * res - cx, (ys + 0.5) * res - cy
            lx = gx * c + gy * s
            ly = -gx * s + gy * c
            grid |= (np.abs(lx) < w / 2) & (np.abs(ly) < h / 2)
    return grid


def map_occupancy(gmap, thresh=0.6):
    return (1.0 / (1.0 + np.exp(-gmap.logodds))) > thresh


def iou(a, b):
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return float(inter / union) if union > 0 else 0.0


def simulate(world, waypoints, duration=45.0, seed=7,
             speed_scale=1.02, yaw_bias=0.0012,
             coarse_window=0.40, fine_window=0.10,
             collect_map=False):
    """复刻 run.py 的建图循环，但参数可调。返回指标字典。"""
    np.random.seed(seed)
    v = 1.2
    max_r = 0.7
    lidar_angles = np.radians(np.linspace(-180.0, 180.0, 181))
    max_range = 8.0

    map_odom = GridMap(size=world.size, res=0.1)
    map_corr = GridMap(size=world.size, res=0.1)

    px, py, pyaw = 2.0, 2.0, 0.0
    ox, oy, oyaw = 2.0, 2.0, 0.0
    corr_x, corr_y, corr_yaw = 2.0, 2.0, 0.0

    dt = 0.1
    n = int(duration / dt)
    scan_interval = 5
    wp_idx = 0
    err_odom, err_corr = [], []
    scan_no = 0
    warmup_scans = 3
    prev_odom = (ox, oy, oyaw)

    for i in range(n):
        wp_idx = next_waypoint(np.array([px, py]), waypoints, wp_idx)
        target = waypoints[wp_idx]
        des_hdg = np.arctan2(target[1] - py, target[0] - px)
        err = (des_hdg - pyaw + np.pi) % (2 * np.pi) - np.pi
        r_cmd = float(np.clip(1.2 * err, -max_r, max_r))

        pyaw += r_cmd * dt
        px += v * np.cos(pyaw) * dt
        py += v * np.sin(pyaw) * dt

        dist = v * dt * speed_scale
        oyaw += (r_cmd + yaw_bias) * dt + np.random.normal(0, 0.003)
        ox += dist * np.cos(oyaw)
        oy += dist * np.sin(oyaw)

        if i % scan_interval == 0:
            ranges = [world.raycast(px, py, pyaw + a, max_range) +
                      np.random.normal(0, 0.01) for a in lidar_angles]
            scan = list(zip(ranges, lidar_angles))

            if scan_no < warmup_scans:
                corr_x, corr_y, corr_yaw = ox, oy, oyaw
            else:
                delta = np.array([ox - prev_odom[0], oy - prev_odom[1]])
                pred_yaw = (corr_yaw + (oyaw - prev_odom[2]) + np.pi) % \
                    (2 * np.pi) - np.pi
                pred = (corr_x + delta[0], corr_y + delta[1], pred_yaw)
                corr_x, corr_y, corr_yaw = scan_match(
                    map_corr, scan, pred, max_range,
                    coarse_window=coarse_window, fine_window=fine_window)
            prev_odom = (ox, oy, oyaw)

            map_odom.update_scan(ox, oy, oyaw, scan, max_range)
            map_corr.update_scan(corr_x, corr_y, corr_yaw, scan, max_range)
            err_odom.append(float(np.hypot(ox - px, oy - py)))
            err_corr.append(float(np.hypot(corr_x - px, corr_y - py)))
            scan_no += 1

    truth = world_grid(world)
    occ_odom = map_occupancy(map_odom)
    occ_corr = map_occupancy(map_corr)
    return {
        "rmse_odom": float(np.sqrt(np.mean(np.square(err_odom)))),
        "rmse_corr": float(np.sqrt(np.mean(np.square(err_corr)))),
        "iou_odom": iou(occ_odom, truth),
        "iou_corr": iou(occ_corr, truth),
        "map_odom": map_odom if collect_map else None,
        "map_corr": map_corr if collect_map else None,
    }


def main():
    os.makedirs("output", exist_ok=True)
    world = World(size=14.0)
    waypoints = np.array([[2.0, 2.0], [12.0, 2.0], [12.0, 12.0],
                          [2.0, 12.0]], dtype=float)

    configs = [
        ("基线 speed=1.02 bias=0.0012", dict()),
        ("距离误差大 speed=1.05", dict(speed_scale=1.05)),
        ("角偏置大 bias=0.003", dict(yaw_bias=0.003)),
        ("搜索窗口小 coarse=0.20", dict(coarse_window=0.20, fine_window=0.05)),
        ("搜索窗口大 coarse=0.80", dict(coarse_window=0.80, fine_window=0.20)),
    ]
    fig_labels = {
        "基线 speed=1.02 bias=0.0012": "baseline",
        "距离误差大 speed=1.05": "dist err 5%",
        "角偏置大 bias=0.003": "yaw bias .003",
        "搜索窗口小 coarse=0.20": "small window",
        "搜索窗口大 coarse=0.80": "large window",
    }

    rows, labels = [], []
    for label, kw in configs:
        t0 = time.time()
        r = simulate(world, waypoints, **kw)
        elapsed = time.time() - t0
        rows.append({
            "config": label,
            "rmse_odom": round(r["rmse_odom"], 3),
            "rmse_corr": round(r["rmse_corr"], 3),
            "iou_odom": round(r["iou_odom"], 3),
            "iou_corr": round(r["iou_corr"], 3),
            "runtime_s": round(elapsed, 1),
        })
        labels.append(fig_labels.get(label, label))
        print("%-28s RMSE 里程计 %.3f / 修正 %.3f   IoU 里程计 %.3f / "
              "修正 %.3f   耗时 %.1fs" %
              (label, r["rmse_odom"], r["rmse_corr"],
               r["iou_odom"], r["iou_corr"], elapsed))

    with open("output/m1_param_sweep.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    x = np.arange(len(labels))
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    ax = axes[0]
    ax.bar(x - 0.2, [r["rmse_odom"] for r in rows], 0.4,
           label="只用里程计", color="#d9534f")
    ax.bar(x + 0.2, [r["rmse_corr"] for r in rows], 0.4,
           label="扫描匹配修正", color="#337ab7")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=20, ha="right", fontsize=8)
    ax.set_ylabel("Trajectory RMSE (m)")
    ax.set_title("Drift / search window vs trajectory accuracy")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.bar(x - 0.2, [r["iou_odom"] for r in rows], 0.4,
           label="只用里程计", color="#d9534f")
    ax.bar(x + 0.2, [r["iou_corr"] for r in rows], 0.4,
           label="扫描匹配修正", color="#337ab7")
    ax.axhline(1.0, color="k", lw=0.8, ls=":")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=20, ha="right", fontsize=8)
    ax.set_ylabel("Map IoU vs ground truth")
    ax.set_title("Mapping quality (higher = closer to truth)")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("output/m1_param_sweep.png", dpi=150)
    plt.close(fig)
    print("已保存 output/m1_param_sweep.csv 和 output/m1_param_sweep.png")


if __name__ == "__main__":
    main()
