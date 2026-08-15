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

注意：gmapping 的消息过滤器依赖 tf1。某些容器/沙箱环境里 tf1 收不到 /tf，
会出现 "MessageFilter Dropped 100%"（本机验证为环境问题，tf2 正常）。
遇到时改用真机 Gazebo 实时建图（turtlebot3 环境已跑通），
存出 gmapping_map.pgm 后再跑本脚本的对比部分；或先跑 replay_live.py
实时重放同一份数据给 gmapping（同样依赖环境 tf1 是否正常）。
"""

import argparse
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
             "_frame_id:=map", "_odom_frame_id:=odom",
             "_base_frame_id:=base_footprint", "_delta:=0.1",
             "_maxUrange:=8.0", "_maxRange:=8.0", "_minimumScore:=30"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(gm)
        time.sleep(4)
        print("[gmapping] slam_gmapping 已启动，回放 bag ...")

        run_cmd("rosbag play --clock --rate 4 %s" % bag_path, env,
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

    os.makedirs("output", exist_ok=True)
    world = World(size=14.0)
    bag_path = os.path.join("output", "m1_synthetic.bag")

    # 1. 生成 bag
    if not os.path.exists(bag_path):
        print("[m1] 生成仿真数据 bag ...")
        run_cmd("python3 make_ros_bag.py", os.environ.copy(), timeout=120)

    # 2. M1 手写 SLAM 处理同一数据（保存地图 + 指标）
    print("[m1] 运行手写 SLAM ...")
    r = simulate(world, np.array([[2.0, 2.0], [12.0, 2.0], [12.0, 12.0],
                                  [2.0, 12.0]], dtype=float),
                 collect_map=True)
    m1_yaml = save_map_pgm(r["map_corr"], os.path.join("output", "m1_map.pgm"))
    truth = world_grid(world)
    m1_iou = iou(map_occupancy(r["map_corr"]), truth)
    print("[m1] 修正后轨迹 RMSE %.3f m，地图与真值 IoU %.3f"
          % (r["rmse_corr"], m1_iou))

    # 3. gmapping 侧
    if args.with_gmapping:
        run_gmapping(bag_path, os.path.join("output", "gmapping_map"))

    # 4. 对比出图
    m1_img = load_pgm(os.path.join("output", "m1_map.pgm"))
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].imshow(np.flipud(m1_img), cmap="gray_r", vmin=0, vmax=255,
                   origin="lower", extent=[0, 14, 0, 14])
    axes[0].set_title("M1 手写 SLAM")

    gm_path = os.path.join("output", "gmapping_map.pgm")
    if os.path.exists(gm_path):
        gm_img = load_pgm(gm_path)
        gm_yaml = os.path.join("output", "gmapping_map.yaml")
        import yaml as _yaml
        with open(gm_yaml) as f:
            meta = _yaml.safe_load(f)
        gm_res = float(meta["resolution"])
        gm_origin = [float(v) for v in meta["origin"][:2]]
        out_shape = (140, 140)
        gm_rs = resample_map(gm_img, gm_res, gm_origin, 0.1, [0.0, 0.0],
                             out_shape)
        m1_rs = resample_map(m1_img, 0.1, [0.0, 0.0], 0.1, [0.0, 0.0],
                             out_shape)
        known = (m1_rs < 254) & (gm_rs < 254)
        agree = np.mean(occupied_mask(m1_rs[known]) ==
                        occupied_mask(gm_rs[known])) if known.any() else 0.0
        gm_iou = iou(occupied_mask(m1_rs), occupied_mask(gm_rs))
        axes[1].imshow(np.flipud(gm_rs), cmap="gray_r", vmin=0, vmax=255,
                       origin="lower", extent=[0, 14, 0, 14])
        axes[1].set_title("gmapping")
        print("[对比] 占据格一致率 %.3f，占据格 IoU %.3f" % (agree, gm_iou))
    else:
        axes[1].text(0.5, 0.5, "gmapping 地图缺失\n用 --with-gmapping 运行",
                     ha="center", va="center")
        axes[1].set_title("gmapping（未运行）")
        print("[对比] 未检测到 gmapping 地图，跳过对比（"
              "运行: python3 compare_gmapping.py --with-gmapping）")

    plt.tight_layout()
    plt.savefig(os.path.join("output", "m1_vs_gmapping.png"), dpi=150)
    plt.close(fig)
    print("已保存 output/m1_vs_gmapping.png")


if __name__ == "__main__":
    main()
