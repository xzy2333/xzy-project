#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M2：2D UUV 航点跟踪（LOS 制导 + PID），带空间变化的洋流扰动。

产出：
  output/m2_trajectory.png  轨迹 + 洋流场
  output/m2_control.png     航向误差 / 速度 / 艏摇指令
"""

import os

import matplotlib
matplotlib.use("Agg")  # 无窗口直接存图；想在电脑上看实时窗口就删掉这行
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Micro Hei",
                                   "SimHei", "Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

from los_pid import LOSGuidance, PID
from uuv_model import UUV2D, wrap_angle


def current_field(x, y):
    """空间变化的洋流：总体向东偏北，带正弦扰动。"""
    return np.array([0.5 + 0.12 * np.sin(0.3 * y),
                     0.15 + 0.08 * np.cos(0.25 * x)])


def main():
    np.random.seed(42)
    os.makedirs("output", exist_ok=True)

    waypoints = np.array([[0.0, 0.0], [9.0, 0.0], [9.0, 7.0],
                          [2.0, 7.0], [2.0, 3.0]], dtype=float)
    boat = UUV2D(x=0.0, y=0.0, yaw=0.3, u=0.0)
    los = LOSGuidance(lookahead=1.5, accept_radius=0.8)

    yaw_pid = PID(1.6, 0.05, 0.35, -1.0, 1.0)
    speed_pid = PID(1.2, 0.10, 0.00, -0.5, 1.5)
    target_speed = 1.2

    dt = 0.05
    t_max = 300.0
    n = int(t_max / dt)

    traj = np.zeros((n, 2))
    err_log = np.zeros(n)          # 航向误差（弧度）
    speed_log = np.zeros(n)        # 实际速度
    cmd_log = np.zeros((n, 2))     # [u_cmd, r_cmd]
    seg_log = np.zeros(n, dtype=int)

    finish_t = None
    i = 0
    for i in range(n):
        p = boat.pos
        traj[i] = p
        seg_log[i] = los.seg_idx

        des_hdg, done = los.desired_heading(p, waypoints)
        err = wrap_angle(des_hdg - boat.yaw)
        err_log[i] = err

        r_cmd = yaw_pid.update(err, dt)
        u_cmd = float(np.clip(speed_pid.update(target_speed - boat.state[3], dt),
                              0.0, 1.6))
        cmd_log[i] = [u_cmd, r_cmd]
        speed_log[i] = boat.state[3]

        boat.step(u_cmd, r_cmd, current_field, dt)

        if done and np.hypot(*(boat.pos - waypoints[-1])) < los.accept_radius:
            finish_t = i * dt
            break

    n_end = i + 1
    if finish_t is None:
        finish_t = n_end * dt

    print("== M2 仿真结果 ==")
    print("到达终点用时: %.1f s" % finish_t)
    print("终点误差: %.3f m" % np.hypot(*(boat.pos - waypoints[-1])))
    print("平均航向误差: %.1f deg" % np.degrees(np.mean(np.abs(err_log[:n_end]))))

    # ---- 图 1：轨迹 + 洋流 ----
    fig, ax = plt.subplots(figsize=(9, 6))
    ax.plot(traj[:n_end, 0], traj[:n_end, 1], "b-", lw=2, label="实际轨迹")
    ax.plot(waypoints[:, 0], waypoints[:, 1], "k--", alpha=0.5, label="航点路径")
    ax.scatter(waypoints[:, 0], waypoints[:, 1], c="k", zorder=5)
    ax.scatter(*waypoints[0], c="g", s=80, marker="o", label="起点")
    ax.scatter(*waypoints[-1], c="r", s=120, marker="*", label="终点")

    xs = np.arange(0.0, 10.0, 1.5)
    ys = np.arange(-1.0, 8.0, 1.5)
    X, Y = np.meshgrid(xs, ys)
    U = np.zeros_like(X)
    V = np.zeros_like(X)
    for r in range(X.shape[0]):
        for c in range(X.shape[1]):
            cu = current_field(X[r, c], Y[r, c])
            U[r, c], V[r, c] = cu
    ax.quiver(X, Y, U, V, alpha=0.35, color="c", width=0.004, label="洋流")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("UUV 航点跟踪（LOS + PID，含洋流） 用时 %.1f s" % finish_t)
    ax.legend(loc="best")
    ax.set_aspect("equal")
    plt.tight_layout()
    plt.savefig("output/m2_trajectory.png", dpi=150)
    plt.close(fig)

    # ---- 图 2：误差与控制 ----
    t_axis = np.arange(n_end) * dt
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(t_axis, np.degrees(err_log[:n_end]))
    axes[0].set_ylabel("航向误差 (deg)")
    axes[0].grid(alpha=0.3)
    axes[1].plot(t_axis, speed_log[:n_end], label="实际速度")
    axes[1].axhline(target_speed, color="r", ls="--", alpha=0.6, label="目标速度")
    axes[1].set_ylabel("速度 (m/s)")
    axes[1].legend()
    axes[1].grid(alpha=0.3)
    axes[2].plot(t_axis, cmd_log[:n_end, 1])
    axes[2].set_ylabel("艏摇指令 r (rad/s)")
    axes[2].set_xlabel("时间 (s)")
    axes[2].grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig("output/m2_control.png", dpi=150)
    plt.close(fig)

    print("已保存 output/m2_trajectory.png 和 output/m2_control.png")


if __name__ == "__main__":
    main()
