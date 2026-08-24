"""底盘控制逻辑闭环仿真 + 纯轮式里程计轨迹可视化。

不经过 diff_drive_controller:cmd_vel -> chassis_driver(麦轮解算+4轮PID,
myrobot_controller 包) -> /wheel_effort_controller/commands -> gz 关节力矩,
反馈取 /joint_states,验证的就是实车要跑的那套控制逻辑。

窗口布局:
  - Gazebo 窗口(headless:=false 时):看车在场地里跑/导航
  - RViz 窗口(rviz:=true 时):看纯轮式里程计累计轨迹(/wheel_odom_path,绿色)

用法(容器内):
  # 带界面(导航时用这个)
  ros2 launch myrobot_gazebo gz_chassis_test.launch.py
  # 服务器模式无 GUI(自动测试用;激光传感器以无头渲染运行)
  ros2 launch myrobot_gazebo gz_chassis_test.launch.py headless:=true rviz:=false

  # 导航(另开终端,地图为场地;注意:必须用 chassis_nav,不要用 nav.launch.py——
  # 那个自带一套 gz_sim 仿真,同时跑会双实例冲突):
  ros2 launch myrobot_bringup chassis_nav.launch.py
  # 手动开车:
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
"""
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('myrobot_gazebo')
    headless = LaunchConfiguration('headless')
    rviz_on = LaunchConfiguration('rviz')

    robot_description = ParameterValue(
        Command(['xacro ', pkg + '/robot/urdf.xacro',
                 ' controller_yaml:=', pkg + '/config/chassis_test_controllers.yaml']),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # headless:=false 打开 Gazebo 图形界面(导航场景);true 仅物理服务器+
    # 无头渲染(传感器仍出数据,自动测试用)
    gz_sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments=[(
            'gz_args',
            [IfElseSubstitution(headless, '-r -s --headless-rendering ', '-r '),
             launch.substitutions.TextSubstitution(text=' ' + pkg + '/worlds/field.world')])],
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'myrobot',
                   '-x', '11', '-y', '11', '-z', '0.2', '-Y', '1.57'],
    )

    # 桥:仿真时钟(odom 时间戳/控制器)+ 激光(导航/避障需要)
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # 真值里程计(URDF odometry 插件发出):打滑检测/跟踪验证,
            # 与 /odom(轮反馈正解算)对比即知轮子是否在空转说谎
            '/model/myrobot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        parameters=[{'use_sim_time': True}],
    )

    jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    effort = Node(
        package='controller_manager', executable='spawner',
        arguments=['wheel_effort_controller'],
    )

    # 被测对象:我们的底盘控制逻辑(gazebo 模式)
    # publish_tf:=true —— 导航/AMCL 需要 odom->base TF;以后接 EKF 时改回 false
    chassis = Node(
        package='myrobot_controller', executable='chassis_driver',
        output='screen',
        parameters=[{
            'mode': 'gazebo',
            'use_sim_time': True,
            'wheel_radius': 0.1,
            'half_diagonal': 0.4,
            'gear_ratio': 19.2,
            'torque_per_current': 0.0003,
            'pid.kp': 1.95,
            'pid.ki': 0.50,
            'publish_tf': True,
            'publish_path': True,
            'joint_names': ['left_front_wheel_joint', 'left_back_wheel_joint',
                            'right_front_wheel_joint', 'right_back_wheel_joint'],
        }],
    )

    # RViz:纯轮式里程计轨迹(/wheel_odom_path)+ 里程计箭头 + 激光
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', pkg + '/config/chassis_path.rviz'],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        condition=IfCondition(rviz_on),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true=仅物理服务器+无头渲染(无 Gazebo 窗口)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否打开 RViz 轨迹可视化窗口'),
        rsp,
        gz_sim,
        spawn,
        bridge,
        rviz,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(
            OnProcessExit(target_action=jsb, on_exit=[effort])),
        RegisterEventHandler(
            OnProcessExit(target_action=effort, on_exit=[chassis])),
    ])
