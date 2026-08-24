import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('myrobot_gazebo')
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', pkg + '/robot/urdf.xacro']),
        value_type=str,
    )

    rsp = launch_ros.actions.Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    teleop = launch.actions.ExecuteProcess(
    cmd=['tilix', '-e',
         'ros2 run teleop_twist_keyboard teleop_twist_keyboard'],
)

    # 新版 Gazebo(Fortress)本体;--headless-rendering 可在无显卡时渲染传感器
    gz_sim = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments=[('gz_args', '-r ' + pkg + '/worlds/field.world')],
    )

    # 在 Gazebo 里生成机器人(替代旧版 spawn_entity.py)
    spawn = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'myrobot', '-x', '11', '-y', '11', '-z', '0.2',      # 出生位置(米)
               '-R', '0', '-P', '0', '-Y', '1.57'],
    )

    # 桥:gz 传输层 ⇄ ROS。桥接条目写在 config/gz_bridge.yaml(单一配置源),
    # Humble 的 parameter_bridge 不认 yaml 文件参数(那是 Jazzy 功能),
    # 所以这里加载后翻译成 CLI 形式 topic@ros_type[gz_type
    import yaml
    with open(pkg + '/config/gz_bridge.yaml') as f:
        bridge_entries = yaml.safe_load(f)
    dir_mark = {'GZ_TO_ROS': '[', 'ROS_TO_GZ': ']', 'BIDIRECTIONAL': '@'}
    bridge_args = [
        f"{e['topic_name']}@{e['ros_type_name']}{dir_mark[e['direction']]}{e['gz_type_name']}"
        for e in bridge_entries]

    bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # EKF:轮式里程计 + IMU 融合,发布 odom→base_link TF
    # (常驻。rosbot 同款:仿真/实车/建图/导航全都用它,配置见 config/ekf.yaml)
    ekf = launch_ros.actions.Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node',
        parameters=[pkg + '/config/ekf.yaml'],
        output='screen')

    # 防撞护栏:激光看路,/cmd_vel → /cmd_vel_safe 减速/刹停(见 myrobot_controller)
    guard = launch_ros.actions.Node(
        package='myrobot_controller', executable='cmd_vel_guard.py',
        name='cmd_vel_guard', output='screen')

    jsb = launch_ros.actions.Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'])
    drive = launch_ros.actions.Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_drive_controller'])
    imu_pub = launch_ros.actions.Node(
        package='controller_manager', executable='spawner',
        arguments=['imu_sensor_broadcaster'])

    return launch.LaunchDescription([
        rsp,
        gz_sim,
        spawn,
        bridge,
        ekf,
        teleop,
        guard,
        launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessExit(
            target_action=spawn, on_exit=[jsb])),
        launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessExit(
            target_action=jsb, on_exit=[drive])),
        launch.actions.RegisterEventHandler(launch.event_handlers.OnProcessExit(
            target_action=drive, on_exit=[imu_pub])),
    ])
