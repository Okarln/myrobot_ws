"""台架导航测试:纯里程计定位(无雷达、无 AMCL),轮子悬空空转即可看到全程。

用途:
  实车架空(轮子离地可自由转动),跑完整 Nav2:在 RViz 里用 2D Goal Pose
  指定目标点,观察
    (1) 车在 SLAM 地图上沿规划路径"前进"——里程计把空转积分成位姿;
    (2) 四个轮子按麦轮逆解的方向/转速转动——/joint_states 驱动 RViz 轮子动画。

出生点(start_x/y/yaw):
  车固定出生在"建图时的那一点",移动距离由里程计积分得出:
  - 默认 field_map(全场真值图,12x12m):出生点 = Gazebo 建图出生位 (11,11,90°),
    已按墙体对齐验证(slam_map 原点经 R(90°)+(11,11) 变换与 field_map 91% 重合);
  - 若用 map:=.../slam_map.yaml(SLAM 部分探索图,原点即建图起点):传 start_x:=0
    start_y:=0 start_yaw:=0。注意该图只有 9.55x6.10m,+x 仅 1.13m/-y 仅 1.16m
    就出图界,目标尽量朝 -x/+y。

与 nav_real.launch.py 的差异:
  - 定位不用 AMCL:map→odom 由 map_tf_adjust 提供(出生点钉扎,
    RViz 2D Pose Estimate 点击可随时重钉扎到车的真实位置);
  - use_lidar:=false:不起雷达链与护栏,wheel_kinematics 直接订 Nav2 的 /cmd_vel;
  - 其余(桥接/运动学/IMU/EKF/Nav2 规划控制)与实车完全一致,测的是真实代码路径。

用法:
  实车台架(主板供电、轮子悬空):
    ros2 launch myrobot_real nav_odom_test.launch.py
  完全离线(没有主板,另开终端先跑 mock):
    ros2 run myrobot_real mock_stm32
    ros2 launch myrobot_real nav_odom_test.launch.py wheel_port:=/tmp/mock_stm32_tty
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('myrobot_real')
    nav2 = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(os.path.expanduser('~'), 'field_sim', 'map', 'field_map.yaml')
    # 纯里程计版参数:costmap 无 obstacle_layer(依赖 /scan_filtered,没雷达时是死配置)
    params_file = os.path.join(pkg, 'config', 'nav2_params_odom.yaml')

    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')

    # 硬件基座:bridge + kinematics + IMU + EKF(odom→base_link 唯一 TF 权威)
    # 注意:Humble 的 IncludeLaunchDescription 只认 [(名字, 值)] 列表,不能传 dict
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg, '/launch/robot.launch.py']),
        launch_arguments=[('use_rviz', 'false'),
                          ('use_lidar', 'false'),  # 无雷达:cmd_vel 直连,不走护栏
                          ('use_imu', 'false'),    # 纯轮速:不起 IMU 链,EKF 只吃 /odom
                          ('wheel_port', LaunchConfiguration('wheel_port'))])

    # 纯里程计定位:map→odom 调整器替代 AMCL——
    # 启动按出生点钉扎;RViz "2D Pose Estimate"(发 /initialpose)点击车的
    # 真实位姿即可重钉扎(静态 TF 点了没反应,故换此节点)
    static_tf = Node(package='myrobot_real', executable='map_tf_adjust',
                     name='map_tf_adjust', output='screen',
                     parameters=[{'start_x': ParameterValue(start_x, value_type=float),
                                  'start_y': ParameterValue(start_y, value_type=float),
                                  'start_yaw': ParameterValue(start_yaw, value_type=float)}])

    # 地图服务:map_server 单独启动 + 专属 lifecycle 管理器(不启动 AMCL)
    map_server = Node(package='nav2_map_server', executable='map_server',
                      name='map_server', output='screen',
                      parameters=[params_file,
                                  {'yaml_filename': LaunchConfiguration('map')}])
    map_lifecycle = Node(package='nav2_lifecycle_manager',
                         executable='lifecycle_manager',
                         name='lifecycle_manager_map',
                         parameters=[{'node_names': ['map_server'], 'autostart': True}])

    # Nav2 规划控制栈(planner/controller/behavior/BT/速度平滑,与 nav_real 同参数)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2, '/launch/navigation_launch.py']),
        launch_arguments=[('use_sim_time', 'false'),
                          ('params_file', params_file),
                          ('use_composition', 'False')])

    rviz = Node(package='rviz2', executable='rviz2', name='rviz',
                arguments=['-d', get_package_share_directory('myrobot_bringup')
                           + '/config/nav.rviz'],
                parameters=[{'use_sim_time': False}], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map,
                              description='地图 yaml(默认全场真值图;换 slam_map 时出生点传 0 0 0)'),
        DeclareLaunchArgument('wheel_port', default_value='/dev/wheel_stm32',
                              description='STM32 串口;离线 mock 时传 /tmp/mock_stm32_tty'),
        DeclareLaunchArgument('start_x', default_value='11',
                              description='出生点 x [m](field_map 建图出生位)'),
        DeclareLaunchArgument('start_y', default_value='11',
                              description='出生点 y [m]'),
        DeclareLaunchArgument('start_yaw', default_value='1.57',
                              description='出生朝向 [rad],field_map 下 1.57=90°'),
        base, static_tf, map_server, map_lifecycle, navigation, rviz,
    ])
