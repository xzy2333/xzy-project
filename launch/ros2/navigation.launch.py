# ROS2 版导航：Nav2（map_server + AMCL + planner/controller）+ 自动给 AMCL 初始位姿
#
# 与 ROS1 的 launch/navigation.launch 对应。ROS1 那边是把 initial_pose_x/y/a 透传给
# amcl.launch；ROS2 官方 navigation2.launch.py 完全不处理初始位姿（grep 无 initial 字样），
# 所以这里自己补：启动延时后用 ros2 topic pub 发 /initialpose，等价于在 RViz 里点
# "2D Pose Estimate"。不发这个，AMCL 不会发布 map→odom，Nav2 一个目标都走不了。
#
# 用法（容器内）：
#   ros2 launch /workspace/launch/ros2/navigation.launch.py \
#       map_file:=/workspace/maps/ros2_xzy_lab.yaml
#   ros2 launch /workspace/launch/ros2/navigation.launch.py open_rviz:=false \
#       initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_a:=0.0
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def _initial_pose_publisher(context):
    """启动 12 秒后连发 3 次 /initialpose（AMCL 刚起来时订阅可能还没建好）。"""
    x = float(context.launch_configurations.get('initial_pose_x', '0.0'))
    y = float(context.launch_configurations.get('initial_pose_y', '0.0'))
    a = float(context.launch_configurations.get('initial_pose_a', '0.0'))
    yaw_q = {'z': math.sin(a / 2.0), 'w': math.cos(a / 2.0)}
    msg = (
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: %f, y: %f, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: %f, w: %f}}, "
        "covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.06,0,0,0, "
        "0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06]}}"
    ) % (x, y, yaw_q['z'], yaw_q['w'])

    cmd = ('for i in 1 2 3; do '
           'ros2 topic pub --once /initialpose '
           'geometry_msgs/msg/PoseWithCovarianceStamped "%s" >/dev/null 2>&1; '
           'sleep 3; done' % msg)
    return [ExecuteProcess(cmd=['bash', '-c', cmd], output='log')]


def generate_launch_description():
    tb3_navigation2 = get_package_share_directory('turtlebot3_navigation2')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    open_rviz = LaunchConfiguration('open_rviz')
    params_file = LaunchConfiguration('params_file')

    declare_args = [
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(REPO_ROOT, 'maps', 'ros2_xzy_lab.yaml'),
            description='地图 yaml（默认 ROS2 版自建世界地图）'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('open_rviz', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(tb3_navigation2, 'param', 'humble', 'waffle_pi.yaml'),
            description='Nav2 参数（TurtleBot3 waffle_pi）'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),
    ]

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_rviz': open_rviz,
            'autostart': 'true',
        }.items())

    return LaunchDescription(declare_args + [
        nav2,
        TimerAction(period=12.0, actions=[OpaqueFunction(function=_initial_pose_publisher)]),
    ])
