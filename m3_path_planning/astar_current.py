# -*- coding: utf-8 -*-
"""网格 A* 路径规划，支持两种成本：
  mode="length"：最短路径（不考虑洋流）
  mode="time"  ：最短时间路径（考虑洋流：顺流快、逆流慢）
"""

import heapq

import numpy as np


class PathPlanner:
    def __init__(self, width, height, res=1.0, base_speed=1.0):
        self.W = width
        self.H = height
        self.res = res
        self.base_speed = base_speed
        self.obstacles = []   # (x, y, r)

    def add_obstacle(self, x, y, r):
        self.obstacles.append((x, y, r))

    def current(self, x, y):
        """洋流场：下部向东强流，上部向西强流（可自行修改做实验）。"""
        cx = 1.2 - 0.12 * y
        cy = 0.1 * np.sin(0.5 * x)
        return np.array([cx, cy])

    def max_current_speed(self):
        """全图最大流速，用于构造可采纳的时间启发函数。"""
        best = 0.0
        ix_max = int(round(self.W / self.res))
        iy_max = int(round(self.H / self.res))
        for ix in range(ix_max + 1):
            for iy in range(iy_max + 1):
                c = self.current(*self.to_xy(ix, iy))
                best = max(best, float(np.hypot(*c)))
        return best

    def blocked(self, x, y):
        for (ox, oy, r) in self.obstacles:
            if (x - ox) ** 2 + (y - oy) ** 2 < (r + 0.3) ** 2:
                return True
        return False

    def to_idx(self, x, y):
        return int(round(x / self.res)), int(round(y / self.res))

    def to_xy(self, ix, iy):
        return ix * self.res, iy * self.res

    def neighbors(self, ix, iy):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = ix + dx, iy + dy
                if 0.0 <= nx * self.res <= self.W and 0.0 <= ny * self.res <= self.H:
                    yield nx, ny

    def edge_time(self, a, b):
        """a、b 是网格索引；时间 = 距离 / (航速 + 洋流在航行方向上的投影)。"""
        p1 = np.array(self.to_xy(*a), dtype=float)
        p2 = np.array(self.to_xy(*b), dtype=float)
        L = float(np.hypot(*(p2 - p1)))
        if L < 1e-9:
            return 0.0
        e = (p2 - p1) / L
        mid = (p1 + p2) / 2.0
        c = self.current(mid[0], mid[1])
        denom = self.base_speed + float(np.dot(c, e))
        if denom < 0.2:   # 逆流太强时的保护，避免时间变成负数
            denom = 0.2
        return L / denom

    def astar(self, start_xy, goal_xy, mode="time"):
        """返回 (路径点列表, 指标字典)；失败返回 (None, None)。"""
        s = self.to_idx(*start_xy)
        g = self.to_idx(*goal_xy)
        if self.blocked(*start_xy) or self.blocked(*goal_xy):
            return None, None

        cmax = self.max_current_speed()

        def heuristic(a):
            p1 = np.array(self.to_xy(*a), dtype=float)
            p2 = np.array(self.to_xy(*g), dtype=float)
            L = float(np.hypot(*(p2 - p1)))
            if mode == "time":
                # 可采纳下界：假设全程洋流都顺向且最大流速
                return L / (self.base_speed + cmax)
            return L

        def edge_cost(a, b):
            if mode == "time":
                return self.edge_time(a, b)
            p1 = np.array(self.to_xy(*a), dtype=float)
            p2 = np.array(self.to_xy(*b), dtype=float)
            return float(np.hypot(*(p2 - p1)))

        came = {}
        gs = {s: 0.0}
        heap = [(heuristic(s), 0.0, s)]
        found = False
        while heap:
            _, gcur, cur = heapq.heappop(heap)
            if gcur > gs[cur]:
                continue
            if cur == g:
                found = True
                break
            for nb in self.neighbors(*cur):
                if self.blocked(*self.to_xy(*nb)):
                    continue
                ng = gcur + edge_cost(cur, nb)
                if nb not in gs or ng < gs[nb]:
                    gs[nb] = ng
                    came[nb] = cur
                    heapq.heappush(heap, (ng + heuristic(nb), ng, nb))

        if not found:
            return None, None

        # 回溯路径
        path = []
        cur = g
        while True:
            path.append(cur)
            if cur == s:
                break
            if cur not in came:
                return None, None
            cur = came[cur]
        path.reverse()

        pts = [self.to_xy(*ix) for ix in path]
        length = 0.0
        time = 0.0
        for k in range(len(path) - 1):
            p1 = np.array(self.to_xy(*path[k]), dtype=float)
            p2 = np.array(self.to_xy(*path[k + 1]), dtype=float)
            length += float(np.hypot(*(p2 - p1)))
            time += self.edge_time(path[k], path[k + 1])
        return pts, {"length": length, "time": time}
