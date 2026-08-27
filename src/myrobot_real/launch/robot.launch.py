"""实车硬件基座栈:STM32 轮速链路 + YbImu + EKF(+ 可选 PAVO2 雷达)。

组合(全部 use_sim_time=false):
  robot_state_publisher   实车 URDF(纯 TF,无仿真)
  wheel_bridge            STM32 host_link 串口桥接(~/Downloads/CAN 固件)
  wheel_kinematics        cmd_vel→四轮rpm;四轮反馈→/odom+/joint_states
  ybimu_driver            YbImu 串口驱动(依赖 YbImuLib,容器内已装)
  imu_filter_madgwick     /imu/data_raw → /imu/data
  ekf_node                轮式里程计+IMU 融合 → odom→base_link TF
  [use_lidar] pavo2_scan_node + scan_filter + cmd_vel_guard
                          雷达 /scan → /scan_filtered;nav2 的 /cmd_vel 经护栏 → /cmd_vel_safe

用法:
  实车:
    ros2 launch myrobot_real robot.launch.py wheel_port:=/dev/wheel_stm32 imu_port:=/dev/myimu use_lidar:=true
  桌面联调(mock 底盘,IMU 为真):
    ros2 run myrobot_real mock_stm32          # 终端1,生成 /tmp/mock_stm32_tty
    ros2 launch myrobot_real robot.launch.py wheel_port:=/tmp/mock_stm32_tty use_lidar:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('myrobot_real')

    wheel_port = LaunchConfiguration('wheel_port')
    imu_port = LaunchConfiguration('imu_port')
    use_imu = LaunchConfiguration('use_imu')
    use_lidar = LaunchConfiguration('use_lidar')
    use_rviz = LaunchConfiguration('use_rviz')
    lidar_ip = LaunchConfiguration('lidar_ip')
    laser_xyz = LaunchConfiguration('laser_xyz')

    robot_description = Command([
        'xacro ', os.path.join(pkg, 'urdf', 'robot_real.xacro'),
        ' laser_xyz:="', laser_xyz, '"'])

    return LaunchDescription([
        DeclareLaunchArgument('wheel_port', default_value='/dev/wheel_stm32',
                              description='STM32 host_link 串口(USB-TTL)'),
        DeclareLaunchArgument('imu_port', default_value='/dev/myimu',
                              description='YbImu 串口(默认 udev 固定名)'),
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='是否启动 YbImu+Madgwick 链(纯轮速台架测试可关)'),
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='是否启动 PAVO2 雷达链'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('lidar_ip', default_value='10.10.10.121'),
        DeclareLaunchArgument('laser_xyz', default_value='0 0 0.12',
                              description='雷达安装位置 x y z [m],需与实车一致'),

        # ── TF:实车 URDF ──────────────────────────────────────
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': False}]),

        # ── 底盘链:桥接 + 运动学 ─────────────────────────────
        Node(package='myrobot_real', executable='wheel_bridge',
             name='wheel_bridge', output='screen',
             arguments=['--port', wheel_port]),
        # 有雷达:走护栏通道 /cmd_vel_safe;无雷达:直接订 /cmd_vel
        # 参数(几何/协方差)统一由 config/wheel_kinematics.yaml 注入,标定改这里
        Node(package='myrobot_real', executable='wheel_kinematics',
             name='wheel_kinematics', output='screen',
             parameters=[os.path.join(pkg, 'config', 'wheel_kinematics.yaml')],
             remappings=[('cmd_vel', '/cmd_vel_safe')],
             condition=IfCondition(use_lidar)),
        Node(package='myrobot_real', executable='wheel_kinematics',
             name='wheel_kinematics', output='screen',
             parameters=[os.path.join(pkg, 'config', 'wheel_kinematics.yaml')],
             condition=UnlessCondition(use_lidar)),

        # ── IMU 链:YbImu → Madgwick(可关,EKF 仅剩轮速源)──
        Node(package='imu_ros2_device', executable='ybimu_driver',
             name='ybimu_driver', output='screen',
             condition=IfCondition(use_imu)),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter_madgwick', output='screen',
             parameters=[os.path.join(pkg, 'config', 'imu_filter.yaml')],
             condition=IfCondition(use_imu)),

        # ── 状态估计:EKF(odom→base_link 唯一 TF 权威)───────
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             parameters=[os.path.join(pkg, 'config', 'ekf_real.yaml')]),

        # ── 雷达链(可选)────────────────────────────────────
        Node(package='pavo2_ros', executable='pavo2_scan_node',
             name='pavo2_scan_node', output='screen',
             condition=IfCondition(use_lidar),
             parameters=[{'lidar_ip': lidar_ip, 'lidar_port': 2368,
                          'frame_id': 'laser_link', 'topic': 'scan',
                          'range_min': 0.10, 'range_max': 30.0}]),
        Node(package='myrobot_controller', executable='scan_filter.py',
             name='scan_filter', output='screen',
             condition=IfCondition(use_lidar)),
        Node(package='myrobot_controller', executable='cmd_vel_guard.py',
             name='cmd_vel_guard', output='screen',
             condition=IfCondition(use_lidar)),

        # ── RViz(可选)──────────────────────────────────────
        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(use_rviz)),
    ])
