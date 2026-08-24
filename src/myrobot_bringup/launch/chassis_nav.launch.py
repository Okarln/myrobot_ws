"""底盘测试栈专用导航叠加(不含仿真!)。

与 nav.launch.py 的区别:nav.launch.py 自带一整套 gz_sim 基座仿真,
只适合单独跑;本 launch 只包含导航部分,叠加在底盘测试栈之上:

  终端 1: ros2 launch myrobot_gazebo gz_chassis_test.launch.py   (仿真+我们的底盘控制)
  终端 2: ros2 launch myrobot_bringup chassis_nav.launch.py      (地图+AMCL+nav2+RViz)

链路:nav2/velocity_smoother 发 /cmd_vel -> chassis_driver 直接订阅;
/scan -> scan_filter -> /scan_filtered -> AMCL/costmap;
AMCL 发 map->odom TF,底盘栈发 odom->base TF。
"""
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory('myrobot_bringup')

    nav2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch', '/bringup_launch.py']),
        launch_arguments=[
            ('map', LaunchConfiguration('map')),
            ('use_sim_time', 'true'),
            ('params_file', bringup + '/config/nav2_params.yaml'),
            ('use_composition', 'False'),
        ])

    # nav2_params 里 AMCL/costmap 订 scan_filtered,原栈缺这个节点
    scan_filter = launch_ros.actions.Node(
        package='myrobot_controller', executable='scan_filter.py',
        name='scan_filter', output='screen')

    # 导航 RViz:地图 + 纯轮式里程计轨迹 + 激光 + 定位
    rviz = launch_ros.actions.Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', bringup + '/config/nav_chassis.rviz'],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='/home/legolas/field_sim/map/slam_map.yaml'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否打开导航 RViz 窗口'),
        scan_filter,
        nav2,
        rviz,
    ])
