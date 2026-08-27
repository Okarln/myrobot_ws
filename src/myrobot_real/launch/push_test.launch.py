"""手推里程计测试:电机零电流卸力,无 PID/EKF,纯编码器测距 + RViz 实时轨迹。

组合:
  robot_state_publisher   实车 URDF(纯 TF)
  wheel_bridge            STM32 桥接(只走反馈链;卸力零电流由 push_odom 下发)
  push_odom               /wheel_state → /push_odom + /push_path + TF + 退出报告
  rviz2                   config/push_test.rviz(轨迹/里程计箭头/轮子动画)

用法:
  实车:
    ros2 launch myrobot_real push_test.launch.py wheel_port:=/dev/wheel_stm32
  桌面联调(验证话题链路,mock 轮速恒 0):
    终端1: ros2 run myrobot_real mock_stm32
    终端2: ros2 launch myrobot_real push_test.launch.py wheel_port:=/tmp/mock_stm32_tty

注意:勿与 robot.launch.py 同跑——wheel_kinematics 的保活零速会把轮子
抱死(速度环闭环),EKF 也会争抢 odom→base_link TF。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('myrobot_real')

    wheel_port = LaunchConfiguration('wheel_port')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = Command(
        ['xacro ', os.path.join(pkg, 'urdf', 'robot_real.xacro')])

    return LaunchDescription([
        DeclareLaunchArgument('wheel_port', default_value='/dev/wheel_stm32',
                              description='STM32 host_link 串口(USB-TTL)'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        # ── TF:实车 URDF ──────────────────────────────────────
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': False}]),

        # ── 桥接:反馈链只读;零电流卸力由 push_odom 发 ────────
        Node(package='myrobot_real', executable='wheel_bridge',
             name='wheel_bridge', output='screen',
             arguments=['--port', wheel_port]),

        # ── 手推里程计(卸力 + 积分 + 轨迹 + 报告)────────────
        Node(package='myrobot_real', executable='push_odom',
             name='push_odom', output='screen'),

        # ── RViz ──────────────────────────────────────────────
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(pkg, 'config', 'push_test.rviz')],
             condition=IfCondition(use_rviz)),
    ])
