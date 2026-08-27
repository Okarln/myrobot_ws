"""实车导航栈:robot.launch.py(硬件基座)+ Nav2(AMCL 定位 + 规划控制)+ RViz。

前置:已完成 SLAM 建图(slam.launch.py 换成实车基座后建图,或手动指定 map)。

用法:
  ros2 launch myrobot_real nav_real.launch.py map:=/path/to/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('myrobot_real')

    default_map = os.path.join(os.path.expanduser('~/myrobot_ws'), 'maps', 'map.yaml')

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg, '/launch/robot.launch.py']),
        launch_arguments=[('use_rviz', 'false')])  # Humble 只认 [(名字, 值)],不能传 dict

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'), '/launch/bringup_launch.py']),
        launch_arguments=[
            ('map', LaunchConfiguration('map')),
            ('use_sim_time', 'false'),
            ('params_file', os.path.join(pkg, 'config', 'nav2_params_real.yaml')),
            ('use_composition', 'False'),
        ])

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', get_package_share_directory('myrobot_bringup')
                           + '/config/nav.rviz'],
                parameters=[{'use_sim_time': False}], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map,
                              description='SLAM 已保存的地图 yaml'),
        base, nav2, rviz,
    ])
