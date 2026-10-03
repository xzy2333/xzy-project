# ROS2 版仿真启动：加载自建世界 worlds/xzy_lab.world，并在 (-2.0, -0.5) 生成 TurtleBot3
#
# 与 ROS1 的 launch/simulation_world.launch 对应，参数名保持一致（world_file/x_pos/y_pos/gui），
# 方便两端对照。区别：ROS2 这边车是从 SDF（带 gazebo_ros 插件）生成，不是 URDF + spawn_model。
#
# 用法（容器内）：
#   ros2 launch /workspace/launch/ros2/sim_xzy_lab.launch.py
#   ros2 launch /workspace/launch/ros2/sim_xzy_lab.launch.py gui:=false      # 无界面
#   ros2 launch /workspace/launch/ros2/sim_xzy_lab.launch.py \
#       robot_sdf:=/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf
#       ↑ 换回官方完整模型（带相机、激光可视化打开）做对比
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 本文件在 <仓库根>/launch/ros2/ 下，往上两级就是仓库根
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def generate_launch_description():
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file = LaunchConfiguration('world_file')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_sdf = LaunchConfiguration('robot_sdf')

    declare_args = [
        DeclareLaunchArgument(
            'world_file',
            default_value=os.path.join(REPO_ROOT, 'worlds', 'xzy_lab.world'),
            description='SDF 世界文件（默认自建实验室）'),
        DeclareLaunchArgument('x_pos', default_value='-2.0',
                              description='车的生成位置 x（与建图起点一致）'),
        DeclareLaunchArgument('y_pos', default_value='-0.5',
                              description='车的生成位置 y（与建图起点一致）'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否开 Gazebo 图形界面（无头自检用 false）'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=os.path.join(
                REPO_ROOT, 'worlds', 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
            description='机器人模型（默认仓库精简版：无相机、关激光可视化，省软件渲染开销）'),
    ]

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui))

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    # 直接调 spawn_entity.py（不再 include 官方 spawn_turtlebot3.launch.py），
    # 这样机器人模型可以用 robot_sdf 参数替换成本仓库的精简版
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'waffle_pi', '-file', robot_sdf,
                   '-x', x_pos, '-y', y_pos, '-z', '0.01'],
        output='screen')

    return LaunchDescription(declare_args + [
        gzserver, gzclient, robot_state_publisher, spawn_robot])
