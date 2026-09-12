#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按固定安全路线驱动小车走遍自建世界（worlds/xzy_lab.world）的两个房间。

用途：自动化建图测试——不用键盘遥控也能复现同一张地图。

用法（仿真已启动、gmapping 已在运行时）：
    source /opt/ros/noetic/setup.bash
    python3 ~/xzy-project/scripts/drive_lab.py

路线（起点 (-2.0,-0.5) 朝 +x，速度 0.25 m/s，转向 0.7 rad/s）：
    西侧房间走一个 2.5 x 2.0 m 矩形 → 从门洞（y≈0 处 1.2 m 宽）进东侧房间
    → 东侧折返 → 原路回到起点。全程约 85 秒，实测不撞障碍。
注意：房间 8 x 6 m，激光量程 3.5 m；改世界布局后请同步调整路线。
"""

import rospy
from geometry_msgs.msg import Twist

# (线速度 m/s, 角速度 rad/s, 持续秒数)
SEGMENTS = [
    (0.25, 0.0, 10.0),
    (0.0, 0.7, 2.25),
    (0.25, 0.0, 8.0),
    (0.0, 0.7, 2.25),
    (0.25, 0.0, 10.0),
    (0.0, 0.7, 2.25),
    (0.25, 0.0, 7.0),
    (0.0, 0.7, 2.25),
    (0.25, 0.0, 16.0),
    (0.0, 0.7, 2.25),
    (0.25, 0.0, 2.5),
    (0.0, 0.7, 2.25),
    (0.25, 0.0, 14.0),
]


def main():
    rospy.init_node("drive_lab", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(1.0)
    rate = rospy.Rate(10)
    for lin, ang, dur in SEGMENTS:
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < dur and not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x = lin
            msg.angular.z = ang
            pub.publish(msg)
            rate.sleep()
    for _ in range(20):          # 停车：补发零速度
        pub.publish(Twist())
        rate.sleep()
    print("drive finished")


if __name__ == "__main__":
    main()
