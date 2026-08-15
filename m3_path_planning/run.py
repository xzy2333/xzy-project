#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M3：洋流场下的 A* 路径规划对比实验。

对比"最短路径"（不管洋流）和"最短时间路径"（顺流省时），
直接对应闫老师 2024 年 Drones 论文的核心思想。

产出：output/m3_compare.png + 终端打印的对比指标
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Micro Hei",
                                   "SimHei", "Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

from astar_current import PathPlanner


def main():
    os.makedirs("output", exist_ok=True)

    planner = PathPlanner(width=30.0, height=20.0, res=1.0, base_speed=1.0)
    planner.add_obstacle(8.0, 5.0, 1.5)
    planner.add_obstacle(14.0, 11.0, 2.0)
    planner.add_obstacle(20.0, 4.0, 1.8)
    planner.add_obstacle(24.0, 14.0, 1.5)
    planner.add_obstacle(10.0, 16.0, 1.2)

    start = (2.0, 2.0)
    goal = (28.0, 17.0)

    path_len, m_len = planner.astar(start, goal, mode="length")
    path_tim, m_tim = planner.astar(start, goal, mode="time")

    print("== M3 对比结果 ==")
    print("最短路径:   长度 %.2f m, 按此路径航行时间 %.2f s" %
          (m_len["length"], m_len["time"]))
    print("最短时间路径: 长度 %.2f m, 按此路径航行时间 %.2f s" %
          (m_tim["length"], m_tim["time"]))
    print("时间节省: %.2f s (%.1f%%)" %
          (m_len["time"] - m_tim["time"],
           100.0 * (m_len["time"] - m_tim["time"]) / m_len["time"]))

    fig, ax = plt.subplots(figsize=(10, 7))
    ax.set_xlim(0, 30)
    ax.set_ylim(0, 20)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("A* 路径规划：最短路径 vs 考虑洋流的最短时间路径")

    # 洋流（每 2m 一个箭头）
    xs = np.arange(0.0, 31.0, 2.0)
    ys = np.arange(0.0, 21.0, 2.0)
    X, Y = np.meshgrid(xs, ys)
    U = np.zeros_like(X)
    V = np.zeros_like(X)
    for r in range(X.shape[0]):
        for c in range(X.shape[1]):
            cu = planner.current(X[r, c], Y[r, c])
            U[r, c], V[r, c] = cu
    ax.quiver(X, Y, U, V, alpha=0.4, color="c", width=0.003)

    # 障碍物
    for (ox, oy, r) in planner.obstacles:
        ax.add_patch(plt.Circle((ox, oy), r, color="gray", alpha=0.55))
        ax.text(ox, oy, "obs", ha="center", va="center", fontsize=7, color="white")

    # 两条路径
    pl = np.array(path_len)
    pt = np.array(path_tim)
    ax.plot(pl[:, 0], pl[:, 1], "b--", lw=2, alpha=0.8,
            label="最短路径  L=%.1fm, T=%.1fs" % (m_len["length"], m_len["time"]))
    ax.plot(pt[:, 0], pt[:, 1], "r-", lw=2.5,
            label="最短时间路径 L=%.1fm, T=%.1fs" % (m_tim["length"], m_tim["time"]))
    ax.scatter(*start, c="g", s=90, marker="o", label="起点")
    ax.scatter(*goal, c="k", s=110, marker="*", label="终点")

    ax.legend(loc="upper left")
    ax.grid(alpha=0.25)
    plt.tight_layout()
    plt.savefig("output/m3_compare.png", dpi=150)
    plt.close(fig)
    print("已保存 output/m3_compare.png")


if __name__ == "__main__":
    main()
