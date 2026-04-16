import os

# 导入ROS2 launch相关模块
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging

def generate_launch_description():
    # 创建LaunchDescription对象，用于收集所有要启动的节点和动作
    ld = LaunchDescription()

    # 定义机器人名称和初始位置
    robots = [
        {'name': 'TB3_1', 'x_pose': '0.0', 'y_pose': '0.0', 'z_pose': 0.01},
        {'name': 'TB3_2', 'x_pose': '-0.5', 'y_pose': '0.5', 'z_pose': 0.01},
        {'name': 'TB3_3', 'x_pose': '-0.5', 'y_pose': '-0.5', 'z_pose': 0.01},
        # ...
        # ...
        ]

    # 设置Turtlebot3模型类型
    TURTLEBOT3_MODEL = 'burger'

    # 声明launch参数：是否使用仿真时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    # 声明launch参数：是否启用机器人驱动节点
    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    # 声明launch参数：是否启用rviz
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    # 获取turtlebot3_multi_robot包的路径
    navigation= get_package_share_directory('navigation')

    # 再次获取包路径并拼接nav2启动文件目录
    package_dir = get_package_share_directory('navigation')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')

    # 声明launch参数：rviz配置文件路径
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    # 获取机器人URDF模型文件路径
    urdf = os.path.join(
        navigation, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    # 声明launch参数：导航参数文件路径
    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    # 将所有声明的参数添加到LaunchDescription中
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)


    #设置 Gazebo 模型路径（需替换为您的实际路径）
    turtlebot3_model_path = os.path.join(
        get_package_share_directory('navigation'),
        'models'
    )
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=turtlebot3_model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    ld.add_action(set_gazebo_model_path)

    # ========== PX4 启动部分 ==========
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value='/home/wjrgfdy/PX4/PX4-Autopilot/',
        description='Path to PX4 Autopilot directory'
    )

    px4_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(__file__), 'px4.launch.py')
        ),
        launch_arguments={
            'px4_dir': LaunchConfiguration('px4_dir')
        }.items()
    )

    ld.add_action(px4_dir_arg)
    ld.add_action(px4_include)

    # ========== 全局地图服务器（与命名空间无关） ==========
    # 定义TF话题重映射，将全局TF话题映射到特定命名空间
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    # 创建全局地图服务器节点
    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('navigation'), 'maps', 'room.yaml'),
                     },],
        remappings=remappings)

    # 创建地图服务器的生命周期管理器节点
    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])


    # 将地图服务器及其生命周期管理器添加到LaunchDescription
    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)


    # ========== 小车生成逻辑（延迟10秒执行，等待PX4 Gazebo就绪） ==========
 
    # 收集所有需要延迟执行的动作
    delayed_actions = []

    # 重新定义TF重映射，确保每个机器人的状态发布器发布到正确的命名空间
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 用于跟踪上一个动作，以实现机器人顺序启动
    last_action = None
    
    # 循环创建并启动每个机器人
    for robot in robots:

        # 定义机器人的命名空间（修正：改为字符串）
        namespace = [ '/' + robot['name'] ]

        # 创建机器人的状态发布器节点
        turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                            'publish_frequency': 10.0}],
            remappings=remappings,
            arguments=[urdf],
        )

        # 创建Gazebo中生成机器人实体的节点
        spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(navigation,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,          # 现在传入的是字符串
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', '0.01', '-Y', '0.0',
                '-unpause',
            ],
            output='screen',
        )

        # 创建机器人导航栈的启动命令
        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={  
                                    'slam': 'False',  # 不使用SLAM，使用已有地图
                                    'namespace': namespace,
                                    'use_namespace': 'True',  # 使用命名空间
                                    'map': '',  # 不使用特定地图文件
                                    'map_server': 'False',  # 不启动地图服务器（已全局启动）
                                    'params_file': params_file,  # 导航参数文件
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',  # 自动启动
                                    'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
                                    )

        # 如果是第一个机器人，直接添加到delayed_actions
        if last_action is None:
            delayed_actions.append(turtlebot_state_publisher)
            delayed_actions.append(spawn_turtlebot3_burger)
            delayed_actions.append(bringup_cmd)

        else:
            # 使用RegisterEventHandler确保下一个机器人的创建在上一个完成后才开始
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,  # 在上一个动作完成后执行
                    on_exit=[spawn_turtlebot3_burger,
                            turtlebot_state_publisher,
                            bringup_cmd],
                )
            )

            delayed_actions.append(spawn_turtlebot3_event)

        # 保存当前动作，供下一个机器人使用
        last_action = spawn_turtlebot3_burger
    ######################

    ######################
    # Rviz和驱动节点启动部分（在所有机器人生成后启动）
    ######################
    for robot in robots:

        namespace = '/' + robot['name']   # 修正为字符串

        # 创建初始位姿发布命令，向initialpose话题发布消息
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-1', '--qos-reliability', 'reliable', namespace + '/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        # 创建Rviz启动命令（条件启动，取决于enable_rviz参数）
        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                                   condition=IfCondition(enable_rviz)
                                    )

        # 创建机器人驱动节点（条件启动，取决于enable_drive参数）
        drive_turtlebot3_burger = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=namespace, output='screen',
            condition=IfCondition(enable_drive),
        )

        # 使用RegisterEventHandler确保rviz等节点在所有机器人生成后启动
        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,  # 在上一个动作（最后一个机器人生成）完成后执行
                on_exit=[initial_pose_cmd, rviz_cmd, drive_turtlebot3_burger],
            )
        )

        # 更新最后一个动作为初始位姿发布，以便后续节点顺序执行
        last_action = initial_pose_cmd

        delayed_actions.append(post_spawn_event)
        # 注意：这里重复添加了declare_params_file_cmd，但重复添加不会产生错误
        # 为避免影响，将其加入延迟列表（但原代码是在这里添加的，我们保持）
        delayed_actions.append(declare_params_file_cmd)
    ######################

    # 创建定时器，延迟10秒执行所有小车生成及后续动作
    timer_action = TimerAction(period=15.0, actions=delayed_actions)

    # 将定时器动作添加到LaunchDescription
    ld.add_action(timer_action)

    # 返回完整的LaunchDescription对象
    return ld