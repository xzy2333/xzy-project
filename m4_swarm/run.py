#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M4：领航-跟随编队仿真。

产出：output/m4_formation.png（轨迹 + 队形快照 + 编队误差收敛曲线）
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Micro Hei",
                                   "SimHei", "Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

from formation import FollowerSwarm, Leader, rot2


def main():
    os.makedirs("output", exist_ok=True)

    waypoints = [[0.0, 0.0], [9.0, 0.0], [9.0, 9.0], [2.0, 9.0], [2.0, 3.0]]
    leader = Leader(waypoints, speed=1.0)
    swarm = FollowerSwarm(n=3)

    dt = 0.05
    t_max = 60.0
    n = int(t_max / dt)
    leader_traj = np.zeros((n, 2))
    follower_traj = np.zeros((n, swarm.n, 2))
    err_log = np.zeros(n)

    snapshots = []   # 用于画队形快照的时间点
    for i in range(n):
        leader_traj[i] = leader.pos
        follower_traj[i] = swarm.positions
        err_log[i] = swarm.formation_error(leader.pos, leader.yaw)
        if i % int(10.0 / dt) == 0:
            snapshots.append((leader.pos.copy(),
                              swarm.positions.copy(),
                              leader.yaw))
        lv = leader.step(dt)
        swarm.step(leader.pos, leader.yaw, lv, dt)

    print("== M4 仿真结果 ==")
    print("最终编队误差: %.3f m" % err_log[-1])
    print("最大编队误差: %.3f m" % np.max(err_log))
    print("领航者终点: (%.2f, %.2f)" % tuple(leader.pos))

    colors = ["tab:orange", "tab:green", "tab:purple"]
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    ax = axes[0]
    wp = np.array(waypoints)
    ax.plot(wp[:, 0], wp[:, 1], "k--", alpha=0.4, label="航点路径")
    ax.plot(leader_traj[:, 0], leader_traj[:, 1], "b-", lw=2, label="领航者")
    for k in range(swarm.n):
        ax.plot(follower_traj[:, k, 0], follower_traj[:, k, 1],
                color=colors[k], lw=1.5, label="跟随者 %d" % (k + 1))
    for (ox, oy, r) in swarm.obstacles:
        ax.add_patch(plt.Circle((ox, oy), r, color="gray", alpha=0.55))
    # 队形快照
    for (lp, fps, ly) in snapshots[1:]:
        poly = np.vstack([lp] + [fps[i] for i in range(swarm.n)])
        ax.plot(poly[:, 0], poly[:, 1], "k-", lw=0.8, alpha=0.5)
        for k in range(swarm.n):
            ax.plot([lp[0], fps[k, 0]], [lp[1], fps[k, 1]],
                    "k-", lw=0.4, alpha=0.35)
    ax.scatter(*waypoints[0], c="g", s=80, marker="o", label="起点")
    ax.scatter(*waypoints[-1], c="r", s=110, marker="*", label="终点")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("领航-跟随编队轨迹（含避障）")
    ax.legend(loc="best", fontsize=8)
    ax.set_aspect("equal")

    ax2 = axes[1]
    t = np.arange(n) * dt
    ax2.plot(t, err_log)
    ax2.set_xlabel("时间 (s)")
    ax2.set_ylabel("平均编队误差 (m)")
    ax2.set_title("编队误差收敛")
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("output/m4_formation.png", dpi=150)
    plt.close(fig)
    print("已保存 output/m4_formation.png")


if __name__ == "__main__":
    main()
