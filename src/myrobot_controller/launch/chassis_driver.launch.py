"""实车底盘驱动 launch。

用法:
  # 实车(USB-TTL 接 STM32 USART10: PG11=RX 接 TX, PG12=TX 接 RX, 共地)
  ros2 launch myrobot_controller chassis_driver.launch.py port:=/dev/ttyUSB0

  # 无硬件闭环自测(节点内一阶模型模拟电机反馈)
  ros2 launch myrobot_controller chassis_driver.launch.py mode:=simulated
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description='USB-TTL 串口设备'),
        DeclareLaunchArgument(
            'mode', default_value='serial',
            description='serial(实车)/ gazebo(PID 力矩仿真闭环)/ '
                        'kinematic(运动学直驱仿真,无轮物理)/ simulated(内置模型自测)'),
        Node(
            package='myrobot_controller',
            executable='chassis_driver',
            output='screen',
            parameters=[{
                'port': port,
                'mode': LaunchConfiguration('mode'),
                # 与实车一致的几何参数,按车体实测修改
                'wheel_radius': 0.0763,
                'half_diagonal': 0.20,
                'gear_ratio': 19.2,
                'pid.kp': 1.95,
                'pid.ki': 0.50,
                'publish_tf': False,
            }],
        ),
    ])
