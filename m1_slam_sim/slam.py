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


def icp_match(map_, scan, p0, max_range, max_iter=12):
    """scan-to-map ICP：把当前扫描的点云与地图里的占据格对齐。

    初始化（里程计增量预测）误差小的时候，ICP 收敛又快又稳，
    这是 Cartographer 局部匹配等真实系统的常用思路。
    两点鲁棒性处理：
      1) 只用当前位置 3m 内的地图点（局部匹配，防止被远处点吸走）
      2) 裁剪式：只保留最近距离 < 0.5m 的对应点（剔除地图还没覆盖的墙）
    """
    prob = 1.0 / (1.0 + np.exp(-map_.logodds))
    iy, ix = np.nonzero(prob > 0.6)
    if len(ix) < 10:
        return p0
    map_pts = np.stack([(ix + 0.5) * map_.res, (iy + 0.5) * map_.res], axis=1)
    # 局部匹配：只看预测位姿附近 3m 的地图点
    near_mask = ((map_pts - np.array([p0[0], p0[1]])) ** 2).sum(axis=1) < 16.0
    map_pts = map_pts[near_mask]
    if len(map_pts) < 10:
        return p0

    body = []
    for r, a in scan:
        if r >= max_range - 1e-6:
            continue
        body.append((r * np.cos(a), r * np.sin(a)))
    if len(body) < 10:
        return p0
    scan_pts = np.array(body)

    c, s = np.cos(p0[2]), np.sin(p0[2])
    R = np.array([[c, -s], [s, c]])
    t = np.array([p0[0], p0[1]])
    last_inliers = 0
    last_mean_res = 1e9

    for _ in range(max_iter):
        world_pts = scan_pts @ R.T + t
        d2 = ((map_pts[:, None, :] - world_pts[None, :, :]) ** 2).sum(axis=2)
        best_d = d2.min(axis=0)
        # 裁剪：只保留最近距离 < 0.35m 的对应点
        inlier = best_d < 0.1225
        if inlier.sum() < 12:
            return p0
        last_inliers = int(inlier.sum())
        last_mean_res = float(best_d[inlier].mean())
        corr = map_pts[d2.argmin(axis=0)][inlier]
        world_pts = world_pts[inlier]

        mu_s = world_pts.mean(axis=0)
        mu_m = corr.mean(axis=0)
        X = world_pts - mu_s
        Y = corr - mu_m
        H = X.T @ Y
        U, _, Vt = np.linalg.svd(H)
        dR = Vt.T @ U.T
        if np.linalg.det(dR) < 0:
            Vt[-1, :] *= -1
            dR = Vt.T @ U.T
        dt = mu_m - dR @ mu_s
        R = dR @ R
        t = dR @ t + dt

    yaw = np.arctan2(R[1, 0], R[0, 0])
    # 防发散：修正距离预测太远就拒绝，回到预测位姿
    moved = np.hypot(t[0] - p0[0], t[1] - p0[1])
    dy = (yaw - p0[2] + np.pi) % (2 * np.pi) - np.pi
    # 保守验收：改动太大 / 内点太少 / 残差太大 都退回预测位姿
    if moved > 0.6:
        return p0
    if abs(dy) > np.radians(15.0):
        return p0
    if last_inliers < 30:
        return p0
    if last_mean_res > 0.04:
        return p0
    return (float(t[0]), float(t[1]), float(yaw))
