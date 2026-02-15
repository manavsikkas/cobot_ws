#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # =========================================
    # Cleanup: kill leftover processes from previous launches
    # =========================================
    cleanup = ExecuteProcess(
        cmd=[
            'sh', '-c',
            'pkill -9 -f "gz sim" 2>/dev/null; '
            'pkill -9 -f "parameter_bridge" 2>/dev/null; '
            'pkill -9 -f "robot_state_publisher" 2>/dev/null; '
            'pkill -9 -f "rviz2" 2>/dev/null; '
            'pkill -9 -f "static_transform_publisher.*map.*odom" 2>/dev/null; '
            'pkill -9 -f "ruby.*gz" 2>/dev/null; '
            'sleep 2'
        ],
        shell=False,
        output='screen',
    )

    # =========================================
    # VMware / Software Rendering Fixes
    # =========================================
    # Ogre 1.x uses GLX which works with LIBGL_ALWAYS_SOFTWARE.
    # Ogre 2.x uses EGL which crashes in VMware because EGL rejects
    # forced software rendering.  Stick with ogre (1.x) for VMs.
    vm_env_vars = [
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('SVGA_VGPU10', '0'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
        SetEnvironmentVariable('MESA_LOADER_DRIVER_OVERRIDE', 'llvmpipe'),
        SetEnvironmentVariable('OGRE_RTT_MODE', 'Copy'),
    ]

    # Get the package directory
    pkg_warebot = get_package_share_directory('warebot')

    # =========================================
    # Gazebo Resource Paths
    # =========================================
    install_share_dir = os.path.dirname(pkg_warebot)
    ros_share_dir = '/opt/ros/jazzy/share'
    gz_resource_path = os.pathsep.join([install_share_dir, ros_share_dir])
    gz_env_vars = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gz_resource_path),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # =========================================
    # Generate SDF from URDF
    # =========================================
    # Spawn from SDF file so the gz_ros2_control plugin is preserved
    # (gz create -topic URDF conversion can drop Gazebo plugins)
    sdf_file = '/tmp/warebot.sdf'
    xacro_path = os.path.join(pkg_warebot, 'description', 'robot.urdf.xacro')
    generate_sdf = ExecuteProcess(
        cmd=[
            'sh', '-c',
            f'xacro {xacro_path} use_sim:=true > /tmp/warebot.urdf '
            f'&& gz sdf -p /tmp/warebot.urdf > {sdf_file}'
        ],
        shell=False,
        output='screen',
    )

    # =========================================
    # Gazebo Simulator
    # =========================================
    default_world = os.path.join(pkg_warebot, 'world', 'empty.world')
    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world, description='World to load'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 --render-engine ogre ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # =========================================
    # Gazebo-ROS Bridge (clock + sensors)
    # =========================================
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Velodyne VLP-16 LiDAR
            '/sensors/velodyne/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/sensors/velodyne/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Intel RealSense D435 depth camera
            '/sensors/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/sensors/realsense/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/sensors/realsense/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/sensors/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
    )

    # =========================================
    # Spawn robot in Gazebo
    # =========================================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=['-file', sdf_file, '-name', 'warebot', '-z', '0.5'],
    )

    # =========================================
    # Robot State Publisher
    # =========================================
    # CRITICAL: RSP publishes robot_description with transient_local QoS.
    # The gz_ros2_control plugin's controller_manager subscribes to this
    # topic.  When it receives the URDF it calls GazeboSimSystem::read()
    # which segfaults if the Gazebo entity joints have not been registered
    # yet.  We therefore gate RSP behind OnProcessExit(spawn_entity) plus
    # an extra delay so the plugin has time to run Configure() and at
    # least one PreUpdate() cycle (which registers the ECM joints).
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_warebot, 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # =========================================
    # Static TF: map -> odom (identity)
    # =========================================
    # Placeholder until a localization node (e.g. AMCL, EKF) is added.
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # =========================================
    # RViz2
    # =========================================
    rviz_config_file = os.path.join(pkg_warebot, 'config', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # =========================================
    # ros2_control Controller Spawners
    # =========================================
    # Sequential: joint_state_broadcaster must be active before diff_drive_controller.
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

    # =========================================
    # Launch sequence (event-driven)
    # =========================================
    # 0s  - Generate SDF, start Gazebo, clock bridge
    # 6s  - Spawn model in Gazebo
    # spawn_exit + 5s  - RSP (publishes robot_description for controller_manager)
    # spawn_exit + 8s  - joint_state_broadcaster
    # jsb_exit         - diff_drive_controller
    # ddc_exit         - RViz (odom frame now available)

    # After spawn_entity exits, wait then start RSP
    after_spawn_rsp = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=5.0, actions=[rsp, static_tf_map_odom])],
        )
    )

    # After spawn_entity exits, wait then start joint_state_broadcaster
    after_spawn_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner])
            ],
        )
    )

    # After joint_state_broadcaster exits, start diff_drive_controller
    after_jsb_ddc = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # After diff_drive_controller spawner exits, start RViz (odom frame exists now)
    after_ddc_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    # After cleanup finishes, start Gazebo + bridge + SDF generation
    after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[generate_sdf, gazebo, gz_bridge,
                     TimerAction(period=6.0, actions=[spawn_entity])],
        )
    )

    return LaunchDescription([
        # Environment
        *vm_env_vars,
        *gz_env_vars,

        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        world_arg,

        # 0. Kill leftover processes from previous launches
        cleanup,

        # 1. After cleanup: generate SDF, launch Gazebo, bridge, spawn
        after_cleanup,

        # 2-5. Event-driven chain (all gated behind spawn_entity completion)
        after_spawn_rsp,     # spawn done + 5s -> RSP
        after_spawn_jsb,     # spawn done + 8s -> joint_state_broadcaster
        after_jsb_ddc,       # jsb done        -> diff_drive_controller
        after_ddc_rviz,      # ddc done        -> RViz (odom frame ready)
    ])
