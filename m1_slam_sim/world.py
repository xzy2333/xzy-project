# -*- coding: utf-8 -*-
"""2D 仿真世界：障碍物 + 射线求交，用来模拟激光雷达。"""

import numpy as np


class World:
    def __init__(self, size=14.0):
        self.size = size
        self.obstacles = []
        self.obstacles.append(("rect", 7.0, 7.0, 4.0, 0.8, 0.3))
        self.obstacles.append(("rect", 7.0, 8.0, 0.8, 4.0, 0.0))
        self.obstacles.append(("circle", 4.0, 10.0, 1.0))
        self.obstacles.append(("circle", 10.0, 4.0, 1.0))
        self.obstacles.append(("rect", 4.0, 4.0, 3.0, 0.7, 0.0))

    def hit(self, x, y):
        for o in self.obstacles:
            if o[0] == "circle":
                _, cx, cy, r = o
                if (x - cx) ** 2 + (y - cy) ** 2 < r * r:
                    return True
            else:
                _, cx, cy, w, h, yaw = o
                dx, dy = x - cx, y - cy
                lx = dx * np.cos(yaw) + dy * np.sin(yaw)
                ly = -dx * np.sin(yaw) + dy * np.cos(yaw)
                if abs(lx) < w / 2.0 and abs(ly) < h / 2.0:
                    return True
        return False

    def raycast(self, x, y, theta, max_range=12.0, step=0.05):
        """从 (x,y) 沿 theta 方向发出射线，返回撞到障碍物的距离。"""
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        d = 0.0
        while d < max_range:
            if self.hit(x + cos_t * d, y + sin_t * d):
                return d
            d += step
        return max_range
