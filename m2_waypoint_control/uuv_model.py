# -*- coding: utf-8 -*-
"""2D UUV 简化运动模型。

状态：x, y, 艏向角 yaw, 前进速度 u
- 速度是"一阶惯性"：u 向指令 u_cmd 指数收敛（tau_u 控制快慢）
- 艏摇角速度 r 直接取指令（限幅），实际可再加一阶惯性
- 位置 = 艇体速度投影 + 洋流速度（洋流是空间位置函数）
"""

import numpy as np


def wrap_angle(a):
    """把角度限制到 [-pi, pi]。"""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class UUV2D:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, u=0.0,
                 tau_u=2.0, tau_r=0.4, max_r=1.0):
        self.state = np.array([x, y, yaw, u], dtype=float)
        self.tau_u = tau_u   # 速度响应时间常数（越大越"笨重"）
        self.tau_r = tau_r   # 艏摇响应时间常数
        self.max_r = max_r   # 最大艏摇角速度 rad/s

    @property
    def pos(self):
        return self.state[:2]

    @property
    def yaw(self):
        return self.state[2]

    def step(self, u_cmd, r_cmd, current, dt):
        """current 可以是 2 元数组，也可以是函数 current(x, y) -> [cx, cy]。"""
        x, y, yaw, u = self.state

        # 速度一阶惯性
        u += (u_cmd - u) / self.tau_u * dt

        # 艏摇角速度（限幅）
        r = float(np.clip(r_cmd, -self.max_r, self.max_r))
        yaw = wrap_angle(yaw + r * dt)

        # 洋流
        if callable(current):
            c = np.asarray(current(x, y), dtype=float)
        else:
            c = np.asarray(current, dtype=float)

        # 合速度 = 艇体速度（沿艏向）+ 洋流
        vx = u * np.cos(yaw) + c[0]
        vy = u * np.sin(yaw) + c[1]
        x += vx * dt
        y += vy * dt

        self.state = np.array([x, y, yaw, u])
        return self.state

