# ROS2 版建图：Cartographer（对应 ROS1 那边 gmapping 的位置）
#
# 用法（容器内）：
#   ros2 launch /workspace/launch/ros2/mapping.launch.py                 # 带 RViz
#   ros2 launch /workspace/launch/ros2/mapping.launch.py open_rviz:=false
#
# 存图（另开一个终端，cartographer 要还活着）：
#   ros2 run nav2_map_server map_saver_cli -f /workspace/maps/ros2_xzy_lab \
#     --ros-args -p save_map_timeout:=30.0 -p free_thresh_default:=0.196 \
#     -p occupied_thresh_default:=0.65
#   （free_thresh_default 0.196 是为了和 ROS1 那边 map_saver 的口径对齐）
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb3_cartographer = get_package_share_directory('turtlebot3_cartographer')

    use_sim_time = LaunchConfiguration('use_sim_time')
    open_rviz = LaunchConfiguration('open_rviz')
    configuration_basename = LaunchConfiguration('configuration_basename')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Gazebo 仿真必须为 true，否则时间戳对不上'),
        DeclareLaunchArgument('open_rviz', default_value='true'),
        DeclareLaunchArgument('configuration_basename', default_value='turtlebot3_lds_2d.lua',
                              description='cartographer 配置（TurtleBot3 激光参数）'),
    ]

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_cartographer, 'launch', 'cartographer.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': open_rviz,
            'configuration_basename': configuration_basename,
        }.items())

    return LaunchDescription(declare_args + [cartographer])
