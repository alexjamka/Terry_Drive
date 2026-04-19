import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('buddy_description')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    xacro_file = os.path.join(pkg, 'urdf', 'buddy.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    nav2_config = os.path.join(pkg, 'config', 'nav2_params_hardware.yaml')

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
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch', 'ydlidar_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(
                os.path.expanduser('~'),
                'buddy_ws/src/ydlidar_ros2_driver/params/X4.yaml')
        }.items(),
    )

    person_detector = Node(
        package='buddy_description',
        executable='buddy_person_detector.py',
        output='screen',
    )

    face_recognizer = Node(
        package='buddy_description',
        executable='buddy_face_recognizer.py',
        output='screen',
    )

    target_tracker = Node(
        package='buddy_description',
        executable='buddy_target_tracker.py',
        output='screen',
    )

    follow_controller = Node(
        package='buddy_description',
        executable='buddy_follow_controller.py',
        output='screen',
    )

    oakd_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oakd_tf',
        arguments=['--frame-id', 'BuddyCameraMount_link',
                   '--child-frame-id', 'oak_d_link'],
        output='screen',
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['--frame-id', 'LidarMount_link',
                   '--child-frame-id', 'laser'],
        output='screen',
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'slam_params_file': os.path.join(
                pkg, 'config', 'mapper_params_hardware.yaml')
        }.items(),
    )
    slam_delayed = TimerAction(period=15.0, actions=[slam_toolbox])

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
    )
    nav2_delayed = TimerAction(period=20.0, actions=[nav2])

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'),
                         'launch', 'foxglove_bridge_launch.xml')
        ),
    )

    return LaunchDescription([
        robot_state_publisher,
        serial_bridge,
        lidar_node,
        laser_tf,
        oakd_tf,
        person_detector,
        face_recognizer,
        target_tracker,
        follow_controller,
        foxglove_bridge,
        slam_delayed,
        nav2_delayed,
    ])
