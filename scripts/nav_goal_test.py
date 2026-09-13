#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""依次向 move_base 发 3 个目标点，自动记录"是否到达 + 耗时"（导航验收用）。

用法（世界 + navigation 栈已启动、车已在起点）：
    source /opt/ros/noetic/setup.bash
    python3 ~/xzy-project/scripts/nav_goal_test.py

默认目标点是自建世界（worlds/xzy_lab.world）里三个可达点（map 坐标）：
    西侧房间北侧 (-1.0, 1.5) → 东侧房间（穿过门洞）(2.0, 0.5) → 西侧南侧 (-3.2, -2.2)
换世界/换地图时改下面的 GOALS；单点超时 120 s。

也可以从命令行直接指定目标点（成对给出 x y）：
    python3 ~/xzy-project/scripts/nav_goal_test.py -1.0 1.5
"""

import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

GOALS = [(-1.0, 1.5), (2.0, 0.5), (-3.2, -2.2)]   # map 坐标


def main():
    args = [float(v) for v in sys.argv[1:]]
    goals = list(zip(args[0::2], args[1::2])) or GOALS
    rospy.init_node("nav_goal_test", anonymous=True)
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    result = {"status": None}

    def cb(msg):
        result["status"] = msg.status.status      # 3=SUCCEEDED, 4=ABORTED

    rospy.Subscriber("/move_base/result", MoveBaseActionResult, cb)
    rospy.sleep(3.0)                              # 等 move_base 就绪

    ok = 0
    for i, (x, y) in enumerate(goals, 1):
        result["status"] = None
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = rospy.Time.now()
        g.pose.position.x = x
        g.pose.position.y = y
        g.pose.orientation.w = 1.0
        t0 = time.time()
        pub.publish(g)
        print("[目标 %d] 发往 (%.1f, %.1f) ..." % (i, x, y), flush=True)
        while result["status"] is None and time.time() - t0 < 120:
            if rospy.is_shutdown():
                return
            rospy.sleep(0.5)
        dt = time.time() - t0
        if result["status"] == 3:
            ok += 1
            print("[目标 %d] 到达，耗时 %.1f s" % (i, dt), flush=True)
        else:
            print("[目标 %d] 未到达（status=%s），耗时 %.1f s"
                  % (i, result["status"], dt), flush=True)
        rospy.sleep(2.0)
    print("结果：%d/%d 个目标到达" % (ok, len(goals)))


if __name__ == "__main__":
    main()
