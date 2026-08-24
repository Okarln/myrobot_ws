"""应用层:基于已建 SLAM 地图的 Nav2 导航(仿真模式)。

组合方式:
  基座栈(myrobot_gazebo/gz_sim.launch.py:仿真+机器人+EKF+护栏+遥控)
    + nav2_bringup(map_server 加载地图 + AMCL 定位 + 规划/控制/行为树)
    + RViz(发目标点用)

注意与建图模式的互斥:导航时不要同时跑 slam_toolbox ——
map→odom 的 TF 只能有一个发布者(导航时是 AMCL)。
"""
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gz = get_package_share_directory('myrobot_gazebo')
    bringup = get_package_share_directory('myrobot_bringup')

    # 建好的地图;重新建图后无需改这里,launch 时 map:=新yaml 即可覆盖
    default_map = '/home/legolas/field_sim/map/slam_map.yaml'

    base = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz + '/launch/gz_sim.launch.py']))

    nav2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/bringup_launch.py']),
        launch_arguments=[
            ('map', launch.substitutions.LaunchConfiguration('map')),
            ('use_sim_time', 'true'),
            ('params_file', bringup + '/config/nav2_params.yaml'),
            ('use_composition', 'False'),
        ])

    rviz = launch_ros.actions.Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', bringup + '/config/nav.rviz'],
        parameters=[{'use_sim_time': True}],
        output='screen')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'map', default_value=default_map,
            description='已保存的 SLAM 地图 yaml'),
        base, nav2, rviz,
    ])
