#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    
    # =========================================
    # VM / Software Rendering Fixes
    # =========================================
    libgl_env = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    ogre_env = SetEnvironmentVariable('OGRE_RTT_MODE', 'Copy')
    mesa_env = SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3')
    gz_render_env = SetEnvironmentVariable('GZ_SIM_RENDER_ENGINE', 'ogre2')
    
    # Get the package directory
    pkg_warebot = get_package_share_directory('warebot')
    
    # =========================================
    # Gazebo Resource/Plugin Paths
    # =========================================
    install_share_dir = os.path.dirname(pkg_warebot)
    gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', install_share_dir)
    ign_resource_path = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', install_share_dir)
    
    # Plugin path for gz_ros2_control - include both system and user paths
    gz_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        '/opt/ros/jazzy/lib:/home/kaneki/ros2_ws/install/gz_ros2_control/lib'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot State Publisher (launched via rsp.launch.py, like prac_bot)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_warebot, 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Joint State Publisher (fallback for RViz if ros2_control fails)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz2
    rviz_config_file = os.path.join(pkg_warebot, 'config', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # World argument
    default_world = os.path.join(pkg_warebot, 'world', 'empty.world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': ['-r -v4 --render-engine ogre ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Generate SDF from URDF so the gz_ros2_control plugin is preserved (create -topic can drop it)
    sdf_file = '/tmp/warebot.sdf'
    xacro_path = os.path.join(pkg_warebot, 'description', 'robot.urdf.xacro')
    generate_sdf = ExecuteProcess(
        cmd=['sh', '-c', f'xacro {xacro_path} use_sim:=true > /tmp/warebot.urdf && gz sdf -p /tmp/warebot.urdf > {sdf_file}'],
        shell=False,
        output='screen',
    )

    # Publish robot_description to topic before spawn so gz_ros2_control plugin gets it (transient_local)
    # RSP republishes too, but plugin loads when model spawns so we ensure message is on topic just before
    publish_robot_description = ExecuteProcess(
        cmd=['python3', '-c', (
            'import rclpy\n'
            'from rclpy.node import Node\n'
            'from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy\n'
            'from std_msgs.msg import String\n'
            'rclpy.init()\n'
            'n = Node("rd_pub")\n'
            'with open("/tmp/warebot.urdf") as f: urdf = f.read()\n'
            'q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)\n'
            'p = n.create_publisher(String, "robot_description", q)\n'
            'p.publish(String(data=urdf))\n'
            'for _ in range(20): rclpy.spin_once(n, timeout_sec=0.1)\n'
            'n.destroy_node()\n'
            'rclpy.shutdown()\n'
        )],
        shell=False,
        output='screen',
    )

    # Spawn robot from SDF (ensures plugin is loaded; -topic URDF conversion may omit it)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=['-file', sdf_file, '-name', 'warebot', '-z', '0.5']
    )

    # =========================================
    # Gazebo-ROS Bridge (clock only)
    # =========================================
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # =========================================
    # ros2_control Controller Spawners (sequential: diff_drive after joint_state_broadcaster)
    # Must activate joint_state_broadcaster first; diff_drive_controller fails to activate if run alone.
    # For manual testing: ros2 run controller_manager spawner joint_state_broadcaster
    #                    then ros2 run controller_manager spawner diff_drive_controller
    # =========================================
    ros2_control_config = os.path.join(pkg_warebot, 'config', 'ros2_control.yaml')
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
            '--param-file', ros2_control_config,
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_diff_drive_controller',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
            '--param-file', ros2_control_config,
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Load diff_drive_controller only after joint_state_broadcaster has started
    delayed_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        # Environment variables (set first)
        libgl_env,
        ogre_env,
        mesa_env,
        gz_render_env,
        gz_resource_path,
        ign_resource_path,
        gz_plugin_path,
        
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        world_arg,
        
        # 0. Generate SDF from URDF (keeps gz_ros2_control plugin; run early so /tmp/warebot.sdf exists)
        generate_sdf,
        
        # 1. Launch Gazebo
        gazebo,
        
        # 2. Bridge clock from Gazebo to ROS
        gz_bridge,
        
        # 3. Robot state publisher (rsp republishes robot_description to topic for controller_manager)
        rsp,
        
        # 4. Joint state publisher (fallback for RViz if ros2_control not ready)
        joint_state_publisher_node,
        
        # 5. Publish robot_description to topic just before spawn (so controller_manager gets it when plugin loads)
        TimerAction(period=3.5, actions=[publish_robot_description]),
        
        # 6. Spawn robot in Gazebo (delayed so Gazebo is up)
        TimerAction(period=4.0, actions=[spawn_entity]),
        
        # 7. RViz (delayed so TF is available)
        TimerAction(period=6.0, actions=[rviz_node]),
        
        # 8. Controller spawners: joint_state_broadcaster first, then diff_drive (after first exits)
        TimerAction(period=10.0, actions=[joint_state_broadcaster_spawner]),
        delayed_diff_drive,
    ])
