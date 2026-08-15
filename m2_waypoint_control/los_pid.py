# -*- coding: utf-8 -*-
"""LOS 视线制导 + PID 控制器。"""

import numpy as np

from uuv_model import wrap_angle


class LOSGuidance:
    """LOS 制导：在"当前航段上再往前看 lookahead 距离"处取目标点，朝它转向。

    前瞻距离越大，转向越平缓（更绕）；越小，越贴航点（可能振荡）。
    """

    def __init__(self, lookahead=1.5, accept_radius=0.8):
        self.lookahead = lookahead
        self.accept_radius = accept_radius
        self.seg_idx = 0   # 当前所在航段（waypoints[seg_idx] -> waypoints[seg_idx+1]）

    def desired_heading(self, pos, waypoints):
        p = np.asarray(pos[:2], dtype=float)
        n = len(waypoints)

        # 到达当前航点就切换下一段
        while self.seg_idx < n - 1:
            b = np.asarray(waypoints[self.seg_idx + 1], dtype=float)
            if np.hypot(*(p - b)) < self.accept_radius:
                self.seg_idx += 1
            else:
                break

        # 已经到最后一个航点：直接朝终点
        if self.seg_idx >= n - 1:
            b = np.asarray(waypoints[-1], dtype=float)
            hdg = np.arctan2(b[1] - p[1], b[0] - p[0])
            return hdg, True

        a = np.asarray(waypoints[self.seg_idx], dtype=float)
        b = np.asarray(waypoints[self.seg_idx + 1], dtype=float)
        seg = b - a
        seg_len = np.hypot(*seg)

        # 当前位置在航段上的投影比例 t
        t = float(np.clip(np.dot(p - a, seg) / (seg_len * seg_len), 0.0, 1.0))
        # 前瞻点：沿航段再往前看 lookahead
        la_t = min(t + self.lookahead / seg_len, 1.0)
        la = a + la_t * seg
        hdg = np.arctan2(la[1] - p[1], la[0] - p[0])
        return hdg, False


class PID:
    """简易 PID，带输出限幅和积分限幅（抗饱和的简化版）。"""

    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, err, dt):
        self.integral = float(np.clip(self.integral + err * dt, -2.0, 2.0))
        d = (err - self.prev_err) / dt if dt > 0 else 0.0
        out = self.kp * err + self.ki * self.integral + self.kd * d
        self.prev_err = err
        return float(np.clip(out, self.out_min, self.out_max))

