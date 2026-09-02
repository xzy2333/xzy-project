#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1 手写 SLAM vs gmapping：同一份仿真数据，两种算法，比地图质量。

流程：
  1. 生成 M1 仿真数据 bag（output/m1_synthetic.bag）
  2. 用 M1 手写 SLAM 处理同一数据，保存地图 output/m1_map.pgm
  3. （--with-gmapping）拉起 gmapping 离线处理 bag，保存 output/gmapping_map.pgm
  4. 对齐两张地图，算占据栅格 IoU，出对比图 output/m1_vs_gmapping.png

用法（已 source /opt/ros/noetic/setup.bash）：
    cd m1_slam_sim && python3 compare_gmapping.py            # 只跑 M1 侧
    cd m1_slam_sim && python3 compare_gmapping.py --with-gmapping

注意：
- 静态变换必须写在 /tf_static（make_ros_bag.py 已处理）：tf1 只把 /tf_static
  当作"任意时刻有效"，写在 /tf 会按时间戳生效，gmapping 的 MessageFilter
  按扫描时间查询时全部外推失败（Dropped 100%）。
- gmapping 的参数名是 map_frame / odom_frame / base_frame（不是 *_frame_id）。
- 回放用较低倍率（--rate 2），避免 /tf 与 /scan 同时间戳时的投递竞态。
"""

import argparse
import csv
import os
import subprocess
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Micro Hei",
                                   "SimHei", "Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

from slam import GridMap
from world import World
from experiment import simulate, map_occupancy, iou, world_grid


def save_map_pgm(gmap, path):
    """把 M1 的栅格地图写成 map_server 格式的 .pgm + .yaml（14m 世界）。"""
    prob = 1.0 / (1.0 + np.exp(-gmap.logodds))
    # 0=占据(黑) 205=空闲(白) 254=未知
    img = np.full(prob.shape, 254, dtype=np.uint8)
    known = prob >= 0.51
    img[known] = np.where(prob[known] >= 0.65, 0, 205)
    img = np.flipud(img)  # pgm 从左上角开始，栅格 y 向上
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())
    yaml_path = path.replace(".pgm", ".yaml")
    with open(yaml_path, "w") as f:
        f.write("image: %s\n" % os.path.basename(path))
        f.write("resolution: 0.1\n")
        f.write("origin: [0.0, 0.0, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
    return yaml_path


def load_pgm(path):
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return img


def resample_map(img, res, origin, out_res, out_origin, out_shape):
    """把任意原点/分辨率的栅格重采样到公共网格（最近邻）。"""
    out = np.full(out_shape, 254.0)
    ys, xs = np.mgrid[0:out_shape[0], 0:out_shape[1]]
    wx = out_origin[0] + (xs + 0.5) * out_res
    wy = out_origin[1] + (ys + 0.5) * out_res
    ix = np.round((wx - origin[0]) / res - 0.5).astype(int)
    iy = np.round((wy - origin[1]) / res - 0.5).astype(int)
    valid = (ix >= 0) & (ix < img.shape[1]) & (iy >= 0) & (iy < img.shape[0])
    out[valid] = img[iy[valid], ix[valid]]
    return out


def occupied_mask(img):
    return img < 100


def align_offset(a, b):
    """在 ±4m（40 格）窗口内穷举平移，找 b 与 a 占据格交集最大的偏移。
    两帧地图的坐标差只可能来自建图起始位姿，量级很小，限界可避免 FFT 伪峰。"""
    max_cells = 40
    best = (0, 0, -1)
    for dr in range(-max_cells, max_cells + 1):
        for dc in range(-max_cells, max_cells + 1):
            s = shift_mask(b, dr, dc)
            score = int((a & s).sum())
            if score > best[2]:
                best = (dr, dc, score)
    return best[0], best[1]


def shift_mask(mask, dr, dc):
    """把 mask 平移 (dr, dc) 个格子（超界部分置 0，不做循环卷绕）。"""
    out = np.zeros_like(mask)
    H, W = mask.shape
    src_r0, src_c0 = max(0, -dr), max(0, -dc)
    dst_r0, dst_c0 = max(0, dr), max(0, dc)
    h = min(H - src_r0, H - dst_r0)
    w = min(W - src_c0, W - dst_c0)
    if h > 0 and w > 0:
        out[dst_r0:dst_r0 + h, dst_c0:dst_c0 + w] = \
            mask[src_r0:src_r0 + h, src_c0:src_c0 + w]
    return out


def dilate_mask(mask, k=2):
    """正方形膨胀 k 格：给墙线留容差，缓解"1 格错位即零重叠"问题。"""
    out = mask.copy()
    n = mask.shape[0]
    for di in range(-k, k + 1):
        for dj in range(-k, k + 1):
            if di == 0 and dj == 0:
                continue
            r0, r1 = max(0, di), min(n, n + di)
            c0, c1 = max(0, dj), min(n, n + dj)
            out[r0:r1, c0:c1] |= mask[r0 - di:r1 - di, c0 - dj:c1 - dj]
    return out


def run_cmd(cmd, env, timeout=60, shell=True):
    return subprocess.run(cmd, shell=shell, env=env, timeout=timeout,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True)


def run_gmapping(bag_path, out_prefix):
    """离线跑 gmapping：roscore + slam_gmapping + rosbag play + map_saver。"""
    env = os.environ.copy()
    env["ROS_MASTER_URI"] = "http://localhost:11311"
    env["ROS_HOME"] = "/tmp/ros_home_gmapping"
    os.makedirs(env["ROS_HOME"], exist_ok=True)

    procs = []
    try:
        r = run_cmd("rostopic list", env, timeout=10)
        if "rosout" not in r.stdout:
            roscore = subprocess.Popen(["roscore"], env=env,
                                       stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL)
            procs.append(roscore)
            for _ in range(30):
                time.sleep(0.5)
                if "rosout" in run_cmd("rostopic list", env,
                                       timeout=5).stdout:
                    break
        print("[gmapping] roscore 就绪")

        run_cmd("rosparam set /use_sim_time true", env, timeout=10)
        gm = subprocess.Popen(
            ["rosrun", "gmapping", "slam_gmapping", "scan:=/scan",
             "_map_frame:=map", "_odom_frame:=odom",
             "_base_frame:=base_footprint", "_delta:=0.1",
             "_maxUrange:=8.0", "_maxRange:=8.0", "_minimumScore:=30",
             "_particles:=80", "_linearUpdate:=0.4", "_angularUpdate:=0.2",
             "_srr:=0.05", "_srt:=0.1", "_str:=0.05", "_stt:=0.1"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(gm)
        time.sleep(4)
        print("[gmapping] slam_gmapping 已启动，回放 bag ...")

        run_cmd("rosbag play --clock --rate 2 %s" % bag_path, env,
                timeout=180)
        time.sleep(3)

        r = run_cmd("rosrun map_server map_saver -f %s" % out_prefix,
                    env, timeout=60)
        if "Map saved" not in r.stdout and os.path.exists(out_prefix + ".pgm"):
            print("[gmapping] 地图已保存（%s）" % (out_prefix + ".pgm"))
        else:
            print(r.stdout[-1000:])
    finally:
        for p in procs:
            p.terminate()
        time.sleep(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--with-gmapping", action="store_true",
                    help="同时运行 gmapping 并对比（需要 ROS 环境）")
    args = ap.parse_args()

    # 默认对比配置（经 E1–E5 对照实验确定，见 EXPERIMENTS.md）：
    #   带边界墙的世界（信息量充足，gmapping 与 m1 都收敛）
    #   激光与车体中心重合（数据与 tf 一致，简化模型）
    #   固定随机种子（可逐位复现）
    # 复现旧的"无墙 + 前装激光"配置：WORLD_WALLS=0 LASER_OFFSET=0.16
    os.environ.setdefault("WORLD_WALLS", "1")
    os.environ.setdefault("LASER_OFFSET", "0.0")
    os.environ.setdefault("SEED", "7")
    seed = int(os.environ["SEED"])

    os.makedirs("output", exist_ok=True)
    world = World(size=14.0)
    bag_path = os.path.join("output", "m1_synthetic.bag")

    # 1. 生成 bag
    if args.with_gmapping or not os.path.exists(bag_path):
        if os.path.exists(bag_path):
            os.remove(bag_path)
        print("[m1] 生成仿真数据 bag ...")
        run_cmd("python3 make_ros_bag.py", os.environ.copy(), timeout=120)

    # 2. M1 手写 SLAM 处理同一数据（保存地图 + 指标）
    print("[m1] 运行手写 SLAM ...")
    r = simulate(world, np.array([[2.0, 2.0], [12.0, 2.0], [12.0, 12.0],
                                  [2.0, 12.0]], dtype=float),
                 collect_map=True, seed=seed)
    save_map_pgm(r["map_corr"], os.path.join("output", "m1_map.pgm"))
    print("[m1] seed=%d 修正后轨迹 RMSE %.3f m"
          % (seed, r["rmse_corr"]))

    # 3. gmapping 侧
    if args.with_gmapping:
        run_gmapping(bag_path, os.path.join("output", "gmapping_map"))

    # 4. 对比出图与指标
    #    坐标系约定：栅格行 0 = y 最小（世界 y 向上）；pgm 行 0 = 顶部，
    #    因此读入后要先 flipud 再进统一的公共网格。
    m1_img = np.flipud(load_pgm(os.path.join("output", "m1_map.pgm")))
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    out_res = 0.1
    lo, hi = -10.0, 24.0
    n = int((hi - lo) / out_res)
    common_origin = [lo, lo]
    # 真值栅格：占据=0（黑）、空闲=255，行 0 = y 最小
    truth = (1 - world_grid(world).astype(np.uint8)) * 255
    truth_rs = resample_map(truth, out_res, [0.0, 0.0], out_res,
                            common_origin, (n, n))
    m1_rs = resample_map(m1_img, out_res, [0.0, 0.0], out_res,
                         common_origin, (n, n))
    axes[0].imshow(m1_rs, cmap="gray_r", vmin=0, vmax=255,
                   origin="lower", extent=[lo, hi, lo, hi])
    axes[0].set_title("M1 hand-written SLAM")

    gm_path = os.path.join("output", "gmapping_map.pgm")
    occ_t = truth_rs < 100
    occ1 = m1_rs < 100
    occ1_al = occ1  # m1 的世界原点 == 真值原点，无需平移

    metrics = {"seed": seed, "m1_rmse_m": round(r["rmse_corr"], 4)}

    def report(name, a, b, key):
        raw = iou(a, b)
        dil = iou(dilate_mask(a, 2), b)
        dil2 = iou(dilate_mask(a, 2), dilate_mask(b, 2))
        metrics[key + "_raw"] = round(raw, 4)
        metrics[key + "_dil2"] = round(dil, 4)
        metrics[key + "_dil2both"] = round(dil2, 4)
        print("  %s: raw IoU=%.3f | 容差2格 IoU=%.3f | 双侧容差 IoU=%.3f"
              % (name, raw, dil, dil2))

    print("[指标] 世界=带墙(14m) 种子=%d" % seed)
    report("m1 vs truth", occ1_al, occ_t, "m1_vs_truth")

    if os.path.exists(gm_path):
        gm_img = np.flipud(load_pgm(gm_path))
        gm_yaml = os.path.join("output", "gmapping_map.yaml")
        import yaml as _yaml
        with open(gm_yaml) as f:
            meta = _yaml.safe_load(f)
        gm_res = float(meta["resolution"])
        gm_origin = [float(v) for v in meta["origin"][:2]]
        gm_rs = resample_map(gm_img, gm_res, gm_origin, out_res,
                             common_origin, (n, n))
        occg = occupied_mask(gm_rs)
        # 以真值为基准做有界对齐（两套 map 原点差只可能来自起始位姿）
        dr_t, dc_t = align_offset(occ_t, occg)
        occg_al = shift_mask(occg, dr_t, dc_t)
        metrics["gm_align_dx_m"] = round(dc_t * out_res, 2)
        metrics["gm_align_dy_m"] = round(dr_t * out_res, 2)
        report("gmapping vs truth (对齐 %.2f,%.2f m)"
               % (dc_t * out_res, dr_t * out_res), occg_al, occ_t,
               "gm_vs_truth")
        dr_m, dc_m = align_offset(occ1_al, occg)
        occg_al_m = shift_mask(occg, dr_m, dc_m)
        report("m1 vs gmapping (对齐 %.2f,%.2f m)"
               % (dc_m * out_res, dr_m * out_res), occ1_al, occg_al_m,
               "m1_vs_gm")
        gm_vis = np.where(occg_al, 0, np.where(gm_rs < 254, 205, 254))
        axes[1].imshow(gm_vis, cmap="gray_r", vmin=0, vmax=255,
                       origin="lower", extent=[lo, hi, lo, hi])
        axes[1].set_title("gmapping (aligned)")
    else:
        axes[1].text(0.5, 0.5, "gmapping 地图缺失\n用 --with-gmapping 运行",
                     ha="center", va="center")
        axes[1].set_title("gmapping（未运行）")
        print("[指标] 未检测到 gmapping 地图，跳过对比（"
              "运行: python3 compare_gmapping.py --with-gmapping）")

    plt.tight_layout()
    plt.savefig(os.path.join("output", "m1_vs_gmapping.png"), dpi=150)
    plt.close(fig)
    csv_path = os.path.join("output", "comparison_metrics.csv")
    write_header = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sorted(metrics))
        if write_header:
            w.writeheader()
        w.writerow(metrics)
    print("指标已追加到 %s" % csv_path)
    print("已保存 output/m1_vs_gmapping.png")


if __name__ == "__main__":
    main()
