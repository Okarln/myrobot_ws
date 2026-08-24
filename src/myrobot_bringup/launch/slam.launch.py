"""应用层:2D 激光 SLAM 建图(仿真模式)。

结构参照 husarion rosbot_ros 的分层方式:
  基座栈(myrobot_gazebo/gz_sim.launch.py,含 EKF/护栏/遥控)
    + slam_toolbox 建图
    + RViz 实时显示

以后接实车:基座栈换成 myrobot_bringup/robot.launch.py(硬件版),本文件的
建图/显示部分原样复用 —— 上层不关心底下是仿真还是实车。
"""
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gz = get_package_share_directory('myrobot_gazebo')

    # 基座栈:仿真 + 机器人 + 桥 + EKF + 护栏 + 遥控(全部在 gz_sim.launch.py 里)
    base = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz + '/launch/gz_sim.launch.py']))

    slam = launch_ros.actions.Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[gz + '/config/slam_toolbox.yaml', {'use_sim_time': True}],
        output='screen')

    rviz = launch_ros.actions.Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', gz + '/config/slam.rviz'],
        parameters=[{'use_sim_time': True}],
        output='screen')

    return launch.LaunchDescription([base, slam, rviz])
