#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用世界文件渲染"真值栅格"，评价一张地图 pgm 的质量（IoU）。

为什么需要：地图"看起来不错"不能写进博客/报告，得给数字。本脚本直接从
worlds/*.world 里的 <model> 位姿与尺寸渲染真值占据栅格，再和 map_saver 保存的
pgm 比对，输出：
  - 原始 IoU（逐格严格比对，对墙厚/错位非常敏感）
  - 双侧容差 IoU（两图各膨胀 2 格=0.1~0.2 m 后再比，反映"几何是否大致一致"）

用法：
    python3 scripts/map_quality.py maps/xzy_lab.pgm maps/xzy_lab.yaml \
            worlds/xzy_lab.world
"""

import re
import sys

import numpy as np
import yaml


def load_pgm(path):
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        return np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)


def parse_models(world_path):
    """返回 [(name, cx, cy, w, h, yaw 或 None(圆柱 r))]。"""
    txt = open(world_path).read()
    out = []
    for name, body in re.findall(
            r"<model name=['\"]([^'\"]+)['\"]>(.*?)</model>", txt, re.S):
        pose = re.search(r"<pose>([^<]+)</pose>", body)
        if not pose:
            continue
        p = [float(v) for v in pose.group(1).split()]
        size = re.search(r"<size>([^<]+)</size>", body)
        radius = re.search(r"<radius>([^<]+)</radius>", body)
        if size:
            w, h, _ = [float(v) for v in size.group(1).split()]
            out.append((name, p[0], p[1], w, h, p[5]))
        elif radius:
            out.append((name, p[0], p[1], float(radius.group(1)), None, None))
    return out


def truth_grid(world_path, res, origin, shape):
    """把世界里的盒子/圆柱渲染成真值占据栅格（行 0 = y 最小）。"""
    grid = np.zeros(shape, dtype=bool)
    ys, xs = np.mgrid[0:shape[0], 0:shape[1]]
    gx = origin[0] + (xs + 0.5) * res
    gy = origin[1] + (ys + 0.5) * res
    for name, cx, cy, w, h, yaw in parse_models(world_path):
        dx, dy = gx - cx, gy - cy
        if h is None:                      # 圆柱
            grid |= (dx ** 2 + dy ** 2) < w ** 2
        else:                              # 盒子（按 yaw 旋转回局部坐标）
            c, s = np.cos(yaw), np.sin(yaw)
            lx = dx * c + dy * s
            ly = -dx * s + dy * c
            grid |= (np.abs(lx) < w / 2) & (np.abs(ly) < h / 2)
    return grid


def dilate(mask, k=2):
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


def iou(a, b):
    return (a & b).sum() / (a | b).sum() if (a | b).any() else 0.0


def main():
    pgm_path, yaml_path, world_path = sys.argv[1:4]
    meta = yaml.safe_load(open(yaml_path))
    res = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"][:2]]
    img = np.flipud(load_pgm(pgm_path))          # 行 0 = y 最小
    truth = truth_grid(world_path, res, origin, img.shape)
    occ = img < 100

    def bbox(mask):
        ys, xs = np.nonzero(mask)
        if len(xs) == 0:
            return "空"
        return "x[%.1f,%.1f] y[%.1f,%.1f] 范围 %.1f×%.1f m" % (
            origin[0] + xs.min() * res, origin[0] + xs.max() * res,
            origin[1] + ys.min() * res, origin[1] + ys.max() * res,
            (xs.max() - xs.min()) * res, (ys.max() - ys.min()) * res)

    print("地图: %s  (%d×%d @ %.3f m, origin %s)" %
          (pgm_path, img.shape[1], img.shape[0], res, origin))
    print("  真值占据格 %d  |  实测占据格 %d" % (truth.sum(), occ.sum()))
    print("  真值 bbox: %s" % bbox(truth))
    print("  实测 bbox: %s" % bbox(occ))
    print("  原始 IoU            = %.3f" % iou(truth, occ))
    print("  双侧容差 2 格 IoU    = %.3f" % iou(dilate(truth, 2), dilate(occ, 2)))


if __name__ == "__main__":
    main()
