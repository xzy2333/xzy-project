#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按真实时间实时重放 M1 仿真数据（替代 rosbag，避免 tf1 离线回放丢帧）。

像真车一样以 10Hz 发布 /scan /odom /tf，给 gmapping 离线建图用。

用法（已 source /opt/ros/noetic/setup.bash，roscore 已启动）：
    python3 replay_live.py
"""

import time

import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from run import next_waypoint
from world import World


def yaw_to_quat(yaw):
    return Quaternion(0.0, 0.0, float(np.sin(yaw / 2.0)),
                      float(np.cos(yaw / 2.0)))


def main():
    rospy.init_node("m1_replay_live")
    rate = rospy.Rate(10)
    scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=1)
    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
    tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=1)
    time.sleep(1.0)  # 等订阅建立

    world = World(size=14.0)
    waypoints = np.array([[2.0, 2.0], [12.0, 2.0], [12.0, 12.0],
                          [2.0, 12.0]], dtype=float)

    v = 1.2
    max_r = 0.7
    speed_scale = 1.02
    yaw_bias = 0.0012
    lidar_angles = np.radians(np.linspace(-180.0, 180.0, 181))
    max_range = 8.0
    dt = 0.1
    duration = 45.0
    n = int(duration / dt)
    scan_interval = 5

    px, py, pyaw = 2.0, 2.0, 0.0
    ox, oy, oyaw = 2.0, 2.0, 0.0
    wp_idx = 0

    # 静态变换 base_footprint -> laser，发一次
    ltf = TransformStamped()
    ltf.header = Header(stamp=rospy.Time.now(), frame_id="base_footprint")
    ltf.child_frame_id = "laser"
    ltf.transform.translation.x = 0.16
    ltf.transform.translation.z = 0.10
    ltf.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
    static_msg = TFMessage()
    static_msg.transforms.append(ltf)
    tf_pub.publish(static_msg)

    for i in range(n):
        stamp = rospy.Time.now()

        wp_idx = next_waypoint(np.array([px, py]), waypoints, wp_idx)
        target = waypoints[wp_idx]
        des_hdg = np.arctan2(target[1] - py, target[0] - px)
        err = (des_hdg - pyaw + np.pi) % (2 * np.pi) - np.pi
        r_cmd = float(np.clip(1.2 * err, -max_r, max_r))

        pyaw += r_cmd * dt
        px += v * np.cos(pyaw) * dt
        py += v * np.sin(pyaw) * dt

        dist = v * dt * speed_scale
        oyaw += (r_cmd + yaw_bias) * dt + np.random.normal(0, 0.003)
        ox += dist * np.cos(oyaw)
        oy += dist * np.sin(oyaw)

        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id="odom")
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = ox
        odom.pose.pose.position.y = oy
        odom.pose.pose.orientation = yaw_to_quat(oyaw)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = r_cmd
        odom_pub.publish(odom)

        tf_msg = TFMessage()
        odom_tf = TransformStamped()
        odom_tf.header = Header(stamp=stamp, frame_id="odom")
        odom_tf.child_frame_id = "base_footprint"
        odom_tf.transform.translation.x = ox
        odom_tf.transform.translation.y = oy
        odom_tf.transform.rotation = yaw_to_quat(oyaw)
        tf_msg.transforms.append(odom_tf)
        tf_pub.publish(tf_msg)

        if i % scan_interval == 0:
            ranges = [world.raycast(px, py, pyaw + a, max_range) +
                      np.random.normal(0, 0.01) for a in lidar_angles]
            scan = LaserScan()
            scan.header = Header(stamp=stamp, frame_id="laser")
            scan.angle_min = float(lidar_angles[0])
            scan.angle_max = float(lidar_angles[-1])
            scan.angle_increment = float(lidar_angles[1] - lidar_angles[0])
            scan.scan_time = 0.5
            scan.range_min = 0.1
            scan.range_max = max_range
            scan.ranges = [float(min(r, max_range)) for r in ranges]
            scan_pub.publish(scan)

        rate.sleep()

    print("重放完成", flush=True)


if __name__ == "__main__":
    main()
