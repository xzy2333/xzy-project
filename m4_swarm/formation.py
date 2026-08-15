# -*- coding: utf-8 -*-
"""领航-跟随编队（2D 运动学模型）。

- 领航者沿航点纯追踪
- 跟随者：跟踪"领航者机体坐标系下的目标队形位置" + 与邻居/障碍物的势场避碰
- 这是"集群协同"的入门版本；闫老师方向的进阶是共识算法、任务分配、全覆盖
"""

import numpy as np


def rot2(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


class Leader:
    def __init__(self, waypoints, speed=1.0):
        self.waypoints = [np.asarray(w, dtype=float) for w in waypoints]
        self.pos = self.waypoints[0].copy()
        self.yaw = 0.0
        self.speed = speed
        self.idx = 0

    def step(self, dt):
        if self.idx < len(self.waypoints) - 1:
            target = self.waypoints[self.idx + 1]
        else:
            target = self.waypoints[-1]
        vec = target - self.pos
        dist = float(np.hypot(*vec))
        if dist < 0.5 and self.idx < len(self.waypoints) - 2:
            self.idx += 1
        self.yaw = float(np.arctan2(vec[1], vec[0]))
        step_v = self.speed * np.array([np.cos(self.yaw), np.sin(self.yaw)])
        self.pos = self.pos + step_v * dt
        return step_v


class FollowerSwarm:
    def __init__(self, n=3, k_formation=1.8, k_rep=3.0, d_rep=1.2, v_max=1.5):
        self.n = n
        self.k_formation = k_formation
        self.k_rep = k_rep
        self.d_rep = d_rep
        self.v_max = v_max
        # 机体坐标系下的队形偏移（领航者正后方呈三角）
        self.offsets = np.array([[0.0, -1.5], [1.5, 0.0], [0.0, 1.5]])
        self.obstacles = [(12.0, 5.0, 1.5), (16.0, 11.0, 1.8), (9.0, 15.0, 1.2)]
        # 初始位置：围绕领航者起点散开
        base = np.array([0.0, 0.0])
        self.positions = np.array([base + off * 1.2 for off in self.offsets])

    def follower_cmd(self, i, leader_pos, leader_yaw, leader_vel, dt):
        p_i = self.positions[i]
        des = leader_pos + rot2(leader_yaw) @ self.offsets[i]
        v = self.k_formation * (des - p_i) + leader_vel

        # 势场避碰：其他跟随者 + 领航者
        all_pos = np.vstack([leader_pos, self.positions])
        for j, p_j in enumerate(all_pos):
            if j == i + 1:  # positions 里第 i 行对应 all_pos 第 i+1 行
                continue
            delta = p_i - p_j
            d = float(np.hypot(*delta))
            if 0 < d < self.d_rep:
                v += self.k_rep * (delta / d) * (1.0 - d / self.d_rep)

        # 障碍物势场
        for (ox, oy, r) in self.obstacles:
            delta = p_i - np.array([ox, oy])
            d = float(np.hypot(*delta))
            if 0 < d < r + self.d_rep:
                v += self.k_rep * (delta / d) * (1.0 - d / (r + self.d_rep))

        # 限速
        spd = float(np.hypot(*v))
        if spd > self.v_max:
            v = v / spd * self.v_max
        return v

    def step(self, leader_pos, leader_yaw, leader_vel, dt):
        new_pos = self.positions.copy()
        for i in range(self.n):
            new_pos[i] = self.positions[i] + self.follower_cmd(
                i, leader_pos, leader_yaw, leader_vel, dt) * dt
        self.positions = new_pos

    def formation_error(self, leader_pos, leader_yaw):
        errs = []
        for i in range(self.n):
            des = leader_pos + rot2(leader_yaw) @ self.offsets[i]
            errs.append(float(np.hypot(*(self.positions[i] - des))))
        return float(np.mean(errs))
