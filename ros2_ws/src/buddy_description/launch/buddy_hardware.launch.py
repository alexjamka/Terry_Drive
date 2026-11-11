import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('buddy_description')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    xacro_file = os.path.join(pkg, 'urdf', 'buddy.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    nav2_config = os.path.join(pkg, 'config', 'nav2_params_hardware.yaml')

    # ------------------------------------------------------------------ args
    #
    # mode picks who drives the base. They are mutually exclusive because each
    # would otherwise fight for /cmd_vel:
    #
    #   wander  - random exploration, obstacle avoidance via buddy_scan_guard
    #   follow  - person following, same guard, plus the perception stack
    #   nav2    - goal-based navigation; Nav2's own costmaps do the avoiding,
    #             so the guard stays out of the way
    #   idle    - sensors, TF and Foxglove only. Good for bringup debugging.
    #
    mode = LaunchConfiguration('mode')
    use_slam = LaunchConfiguration('use_slam')

    is_wander = IfCondition(PythonExpression(["'", mode, "' == 'wander'"]))
    is_follow = IfCondition(PythonExpression(["'", mode, "' == 'follow'"]))
    is_nav2 = IfCondition(PythonExpression(["'", mode, "' == 'nav2'"]))
    # The reactive behaviours share the guard.
    is_reactive = IfCondition(
        PythonExpression(["'", mode, "' in ('wander', 'follow')"]))
    slam_on = IfCondition(PythonExpression(["'", use_slam, "' == 'true'"]))

    args = [
        DeclareLaunchArgument(
            'mode', default_value='idle',
            choices=['idle', 'wander', 'follow', 'nav2'],
            description='Who drives the base.'),
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            choices=['true', 'false'],
            description='Run slam_toolbox. Required for mode:=nav2.'),
    ]

    # ------------------------------------------------------------ always-on
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        output='screen',
    )

    serial_bridge = Node(
        package='buddy_description',
        executable='buddy_serial_bridge.py',
        output='screen',
    )

    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ydlidar_ros2_driver'),
                         'launch', 'ydlidar_launch.py')
        ),
        # Our own copy, so bringup doesn't depend on where the third-party
        # driver source happens to be checked out.
        launch_arguments={
            'params_file': os.path.join(pkg, 'config', 'ydlidar_x4.yaml')
        }.items(),
    )

    # Lidar mounting yaw. buddy_scan_guard and buddy_wanderer read this out of
    # TF to interpret scans in the robot frame, so it has to be right or every
    # avoidance decision is mirrored.
    #
    # Calibrated 2026-11-11 against a known landmark: the 40x40 mm extrusion
    # behind the lidar shows up at 0.117 m, and it lands at laser-frame 180 deg.
    # Since that extrusion is physically at the robot's REAR, laser 0 deg points
    # forward and the offset is zero. This was previously 3.14159, which mapped
    # the rear extrusion onto the FRONT sector and inverted obstacle avoidance.
    laser_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='laser_tf',
        arguments=['--frame-id', 'LidarMount_link', '--child-frame-id', 'laser',
                   '--yaw', '0.0'],
        output='screen',
    )

    oakd_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='oakd_tf',
        arguments=['--frame-id', 'BuddyCameraMount_link',
                   '--child-frame-id', 'oak_d_link'],
        output='screen',
    )

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'),
                         'launch', 'foxglove_bridge_launch.xml')
        ),
    )

    # ------------------------------------------------- reactive drive stack
    scan_guard = Node(
        package='buddy_description', executable='buddy_scan_guard.py',
        output='screen', condition=is_reactive,
    )

    wanderer = Node(
        package='buddy_description', executable='buddy_wanderer.py',
        output='screen', condition=is_wander,
    )

    # Perception is only needed to follow someone, and it is the most expensive
    # thing on the Jetson - keep it off while wandering.
    person_detector = Node(
        package='buddy_description', executable='buddy_person_detector.py',
        output='screen', condition=is_follow,
    )
    face_recognizer = Node(
        package='buddy_description', executable='buddy_face_recognizer.py',
        output='screen', condition=is_follow,
    )
    target_tracker = Node(
        package='buddy_description', executable='buddy_target_tracker.py',
        output='screen', condition=is_follow,
    )
    follow_controller = Node(
        package='buddy_description', executable='buddy_follow_controller.py',
        output='screen', condition=is_follow,
        # Route through the guard instead of straight at the motors.
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
    )

    # ------------------------------------------------------- mapping / nav2
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'slam_params_file': os.path.join(pkg, 'config',
                                             'mapper_params_hardware.yaml')
        }.items(),
        condition=slam_on,
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
            'params_file': nav2_config,
        }.items(),
        condition=is_nav2,
    )

    # Staggered so the TF tree and /scan are alive before anything latches on.
    slam_delayed = TimerAction(period=15.0, actions=[slam_toolbox])
    nav2_delayed = TimerAction(period=20.0, actions=[nav2])

    return LaunchDescription(args + [
        robot_state_publisher,
        serial_bridge,
        lidar_node,
        laser_tf,
        oakd_tf,
        foxglove_bridge,
        scan_guard,
        wanderer,
        person_detector,
        face_recognizer,
        target_tracker,
        follow_controller,
        slam_delayed,
        nav2_delayed,
    ])
