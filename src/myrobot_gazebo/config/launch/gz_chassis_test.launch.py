"""底盘控制逻辑仿真 + 纯轮式里程计轨迹可视化(双链路,control 参数切换)。

不经过 diff_drive_controller:cmd_vel -> chassis_driver(麦轮解算,myrobot_controller 包),
之后二选一:

  control:=kinematic(默认) 运动学直驱,不走轮子物理:
      目标轮速(一阶响应) -> 正解算车体速度 -> /model/myrobot/cmd_vel ->
      gz VelocityControl 插件每步直接设置模型速度。
      无 PID/力矩/摩擦参与,横移可用,跑得稳,适合导航/SLAM 链路联调。
      注意:此模式 z 向速度被钉 0,出生高度取静平衡 0.1m;gz 窗口里轮子不转
      (RViz 里转,chassis_driver 自发 /joint_states)。

  control:=effort  物理闭环(原链路,验证实车控制算法用):
      4 轮 PID 电流 -> 力矩 -> /wheel_effort_controller/commands -> gz 关节力矩,
      反馈取 /joint_states。已知局限:简化轮模型下横移(vy)无法产生侧向力。

窗口布局:
  - Gazebo 窗口(headless:=false 时):看车在场地里跑/导航
  - RViz 窗口(rviz:=true 时):看纯轮式里程计累计轨迹(/wheel_odom_path,绿色)

用法(容器内):
  # 带界面(导航时用这个)
  ros2 launch myrobot_gazebo gz_chassis_test.launch.py
  # 服务器模式无 GUI(自动测试用;激光传感器以无头渲染运行)
  ros2 launch myrobot_gazebo gz_chassis_test.launch.py headless:=true rviz:=false
  # 切回 PID 物理闭环
  ros2 launch myrobot_gazebo gz_chassis_test.launch.py control:=effort
  # 纯运动学(轮速瞬时到位,无电机响应滞后)
  ros2 launch myrobot_gazebo gz_chassis_test.launch.py motor_tau:=0.0

  # 导航(另开终端,地图为场地;注意:必须用 chassis_nav,不要用 nav.launch.py——
  # 那个自带一套 gz_sim 仿真,同时跑会双实例冲突):
  ros2 launch myrobot_bringup chassis_nav.launch.py
  # 手动开车:
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
"""
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, IfElseSubstitution, LaunchConfiguration, \
    PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('myrobot_gazebo')
    headless = LaunchConfiguration('headless')
    rviz_on = LaunchConfiguration('rviz')

    # PythonExpression 产出 'True'/'False' 字符串,再交给 IfCondition/IfElseSubstitution
    is_kinematic = PythonExpression(
        ["'", LaunchConfiguration('control'), "' == 'kinematic'"])
    is_effort = PythonExpression(
        ["'", LaunchConfiguration('control'), "' == 'effort'"])

    robot_description = ParameterValue(
        Command(['xacro ', pkg + '/robot/urdf.xacro',
                 ' controller_yaml:=', pkg + '/config/chassis_test_controllers.yaml',
                 # kinematic 链加载 VelocityControl 插件(直驱模型速度)
                 ' kinematic_move:=', IfElseSubstitution(is_kinematic, 'true', 'false')]),
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

    # kinematic 链 z 向速度被插件钉 0(重力失效),出生高度必须取静平衡 0.1m;
    # effort 链从 0.2m 落地
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'myrobot',
                   '-x', '11', '-y', '11',
                   '-z', IfElseSubstitution(is_kinematic, '0.1', '0.2'),
                   '-Y', '1.57'],
    )

    # 桥:仿真时钟(odom 时间戳/控制器)+ 激光(导航/避障需要)
    # + 车体速度指令 ROS->GZ(kinematic 链用;effort 链无订阅者,空转无害)
    # + 真值里程计(URDF odometry 插件发出):打滑检测/跟踪验证,
    #   与 /odom(轮反馈正解算)对比即知轮子是否在空转说谎
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/myrobot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/myrobot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # effort 链专用控制器(kinematic 链不需要:无轮子执行器接口要 claim,
    # /joint_states 由 chassis_driver 自己发)
    jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        condition=IfCondition(is_effort),
    )
    effort = Node(
        package='controller_manager', executable='spawner',
        arguments=['wheel_effort_controller'],
        condition=IfCondition(is_effort),
    )

    # 被测对象:我们的底盘控制逻辑(kinematic 直驱 / gazebo PID 闭环)
    # publish_tf:=true —— 导航/AMCL 需要 odom->base TF;以后接 EKF 时改回 false
    chassis = Node(
        package='myrobot_controller', executable='chassis_driver',
        output='screen',
        parameters=[{
            'mode': IfElseSubstitution(is_kinematic, 'kinematic', 'gazebo'),
            'use_sim_time': True,
            'wheel_radius': 0.1,
            'half_diagonal': 0.4,
            'gear_ratio': 19.2,
            'motor_tau': ParameterValue(LaunchConfiguration('motor_tau'),
                                         value_type=float),
            'torque_per_current': 0.0003,
            'pid.kp': 1.95,
            'pid.ki': 0.50,
            'publish_tf': True,
            'publish_path': True,
            'joint_names': ['left_front_wheel_joint', 'left_back_wheel_joint',
                            'right_front_wheel_joint', 'right_back_wheel_joint'],
        }],
        # kinematic 模式:正解算出的车体速度 -> VelocityControl 插件
        remappings=[('kinematic_cmd_vel', '/model/myrobot/cmd_vel')],
    )

    # RViz:纯轮式里程计轨迹(/wheel_odom_path)+ 里程计箭头 + 激光
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', pkg + '/config/chassis_path.rviz'],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        condition=IfCondition(rviz_on),
    )

    def chassis_start_chain(context):
        # effort 链等 effort 控制器加载完再启动;kinematic 链不依赖任何控制器,
        # 模型一生成就启动。launch 阶段才能读到 control 的值,所以用 OpaqueFunction。
        if LaunchConfiguration('control').perform(context) == 'effort':
            return [RegisterEventHandler(
                OnProcessExit(target_action=effort, on_exit=[chassis]))]
        return [RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[chassis]))]

    return LaunchDescription([
        DeclareLaunchArgument(
            'control', default_value='kinematic', choices=['kinematic', 'effort'],
            description='kinematic=运动学直驱(默认,无轮物理,横移可用);'
                        'effort=PID 力矩物理闭环(原链路)'),
        DeclareLaunchArgument(
            'motor_tau', default_value='0.05',
            description='kinematic 模式理想执行器一阶响应时间常数 [s],0=瞬时到位'),
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
        OpaqueFunction(function=chassis_start_chain),
        RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(
            OnProcessExit(target_action=jsb, on_exit=[effort])),
    ])
