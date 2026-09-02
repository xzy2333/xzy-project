# -*- coding: utf-8 -*-
"""占位栅格地图（Occupancy Grid Map）+ 相关扫描匹配（Correlative Scan Matching）。

这是 2D SLAM 的最小闭环：
  激光端点撞到墙 → 栅格 log-odds 增加（占据）
  射线穿过的地方 → log-odds 减少（空闲）
  里程计给"猜的位姿" → 扫描匹配在附近搜一个"地图分数最高"的位姿
"""

import numpy as np


class GridMap:
    def __init__(self, size=20.0, res=0.1):
        self.size = size
        self.res = res
        self.n = int(size / res)
        self.logodds = np.zeros((self.n, self.n))
        self.lo_hit = 1.5     # 占据更新量
        self.lo_miss = -0.2   # 空闲更新量
        self.min_lo = -3.0
        self.max_lo = 6.0

    def cell(self, x, y):
        return int(x / self.res), int(y / self.res)

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.n and 0 <= iy < self.n

    def prob_at(self, x, y):
        """取某个世界坐标点的占据概率（未观测 = 0.5）。"""
        ix, iy = self.cell(x, y)
        if not self.in_bounds(ix, iy):
            return 0.5
        return 1.0 / (1.0 + np.exp(-self.logodds[iy, ix]))

    def update_beam(self, x, y, theta, r, max_range):
        """把一束激光写进地图：穿过区域→空闲，端点→占据。"""
        # 关键细节：只把"激光到达之前"的区域标记为空闲，
        # 给墙留出端点一格余量，避免斜射时把薄墙"擦掉"
        # 留 0.4m 余量（约 4 个栅格），防止斜射把薄墙"擦掉"
        free_dist = max(0.0, r - 0.4)
        fx = x + np.cos(theta) * free_dist
        fy = y + np.sin(theta) * free_dist
        dist = free_dist
        steps = max(2, int(dist / (self.res * 0.5)))
        xs = np.linspace(x, fx, steps + 1)
        ys = np.linspace(y, fy, steps + 1)
        for k in range(steps):
            ix, iy = self.cell(xs[k], ys[k])
            if self.in_bounds(ix, iy):
                # 保护已建好的墙：强占据格不再接受空闲更新
                if self.logodds[iy, ix] < 0.5:
                    self.logodds[iy, ix] = np.clip(
                        self.logodds[iy, ix] + self.lo_miss,
                        self.min_lo, self.max_lo)
        if r < max_range - 1e-6:
            end_x = x + np.cos(theta) * r
            end_y = y + np.sin(theta) * r
            ix, iy = self.cell(end_x, end_y)
            if self.in_bounds(ix, iy):
                self.logodds[iy, ix] = np.clip(
                    self.logodds[iy, ix] + self.lo_hit, self.min_lo, self.max_lo)

    def update_scan(self, x, y, yaw, scan, max_range):
        for r, a in scan:
            self.update_beam(x, y, yaw + a, r, max_range)

    def blurred_image(self, passes=2):
        """占据概率图 + 3x3 盒式模糊。

        模糊让墙"变厚"，扫描匹配时产生平滑的分数梯度，
        不容易陷入错误的局部最优（真实 SLAM 里也有类似处理）。
        """
        img = 1.0 / (1.0 + np.exp(-self.logodds))
        for _ in range(passes):
            p = np.pad(img, 1, mode="edge")
            img = (p[:-2, :-2] + p[:-2, 1:-1] + p[:-2, 2:] +
                   p[1:-1, :-2] + p[1:-1, 1:-1] + p[1:-1, 2:] +
                   p[2:, :-2] + p[2:, 1:-1] + p[2:, 2:]) / 9.0
        return img


def score_pose_img(img, scan, pose, max_range, res):
    """候选位姿的打分：激光端点尽量落在墙（占据>0.5）上，
    尽量不落在空地上；未知区域（=0.5）不贡献分数。"""
    x, y, th = pose
    total = 0.0
    for r, a in scan:
        if r >= max_range - 1e-6:
            continue
        ex = x + r * np.cos(th + a)
        ey = y + r * np.sin(th + a)
        ix, iy = int(ex / res), int(ey / res)
        if 0 <= ix < img.shape[1] and 0 <= iy < img.shape[0]:
            total += img[iy, ix] - 0.5
    return total


def scan_match(map_, scan, p0, max_range,
               coarse_window=0.40, fine_window=0.10):
    """相关扫描匹配：粗搜（0.2m/2°）→ 精搜（0.05m/0.5°）。

    coarse_window / fine_window 可调（默认与原来一致），
    用于 M1 参数实验：窗口太小容易跟丢，太大搜索慢。
    """
    img = map_.blurred_image(passes=1)
    res = map_.res

    def search(best_p, best_s, window, dth_deg, step, dth_step):
        xs = np.arange(-window, window + 1e-9, step)
        ys = np.arange(-window, window + 1e-9, step)
        ths = np.radians(np.arange(-dth_deg, dth_deg + 1e-9, dth_step))
        for dx in xs:
            for dy in ys:
                for dth in ths:
                    p = (p0[0] + dx, p0[1] + dy, p0[2] + dth)
                    s = score_pose_img(img, scan, p, max_range, res)
                    if s > best_s:
                        best_s = s
                        best_p = p
        return best_p, best_s

    best_p, best_s = p0, -1e9
    best_p, best_s = search(best_p, best_s, coarse_window, 8.0, 0.20, 2.0)
    best_p, best_s = search(best_p, best_s, fine_window, 2.0, 0.05, 0.5)
    return best_p


