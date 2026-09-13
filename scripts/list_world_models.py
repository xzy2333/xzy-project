#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""列出 .world 文件里所有模型的名称、位姿和尺寸，方便手动调整障碍位置。

用法：
    python3 ~/xzy-project/scripts/list_world_models.py [world文件]
    # 缺省 world 文件：~/xzy-project/worlds/xzy_lab.world

输出示例：
    wall_south        pose=0 -3.0 0.4 0 0 0        size=8.3 0.15 0.8
    crate_1           pose=2.5 -1.8 0.3 0 0 0.3    size=0.8 0.8 0.6

改障碍位置：编辑 world 文件里对应 <model> 的 <pose>x y z roll pitch yaw</pose>，
改完用 `gz sdf -k <world>` 做语法检查，重启仿真即可生效。
"""

import os
import re
import sys


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        "~/xzy-project/worlds/xzy_lab.world")
    with open(path) as f:
        txt = f.read()

    print("== %s ==" % path)
    print("%-16s %-32s %s" % ("模型名", "位姿 pose(x y z roll pitch yaw)",
                              "尺寸"))
    for name, body in re.findall(r"<model name='([^']+)'>(.*?)</model>",
                                 txt, re.S):
        pose = re.search(r"<pose>([^<]+)</pose>", body)
        size = re.search(r"<size>([^<]+)</size>", body)
        radius = re.search(r"<radius>([^<]+)</radius>", body)
        length = re.search(r"<length>([^<]+)</length>", body)
        dim = size.group(1).strip() if size else (
            "圆柱 r=%s h=%s" % (radius.group(1).strip(),
                                length.group(1).strip())
            if radius and length else "-")
        print("%-16s %-32s %s" % (name,
                                  pose.group(1).strip() if pose else "-", dim))


if __name__ == "__main__":
    main()
