"""
RGB-D SLAM on the OAK-D-LITE, via rtabmap.

Run ALONGSIDE buddy_hardware.launch.py with use_slam:=false, e.g.

    ros2 launch buddy_description buddy_hardware.launch.py mode:=teleop use_slam:=false
    ros2 launch buddy_description buddy_vslam.launch.py

The hardware launch supplies the TF tree, the motors and the lidar safety
guard; this one supplies the map. Exactly one of slam_toolbox and rtabmap may
run at a time, because both publish map->odom and they would fight.

Why rtabmap rather than slam_toolbox: the lidar returns nothing past 2.51 m
(measured), which is shorter than the loop-closure search radius, so loops
never closed and rooms got drawn twice. rtabmap closes loops by RECOGNISING a
place from its appearance, which does not depend on sensor range.

Deliberately uses the SAME wheel odometry slam_toolbox had (odom_frame_id),
so a comparison against the lidar maps changes only the sensor. Visual
odometry is the next step if wheel odometry proves to be the limit - the
wheels over-report distance by ~18%, which rtabmap must correct for.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('buddy_description')
    depthai_prefix = get_package_share_directory('depthai_ros_driver')
    params_file = os.path.join(pkg, 'config', 'oakd_rtabmap.yaml')

    name = 'oak'
    localization = LaunchConfiguration('localization')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_prefix, 'launch', 'camera.launch.py')),
        launch_arguments={'name': name, 'params_file': params_file}.items(),
    )

    rtabmap_params = [{
        # Pose the map in ROBOT terms, not camera terms, so the grid lines up
        # with the URDF and Nav2's footprint.
        'frame_id': 'BuddyBaseFootprint_link',
        'map_frame_id': 'map',
        # Take odometry from TF (the serial bridge publishes odom->base), the
        # same source slam_toolbox used.
        'odom_frame_id': 'odom',
        'subscribe_rgb': True,
        'subscribe_depth': True,
        'subscribe_scan': False,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'queue_size': 30,
        'use_sim_time': False,
        'publish_tf': True,
        # The robot drives on a floor; solving in full 6DOF lets the map tilt
        # and roll on bad frames, which a wheeled indoor robot never does.
        'Reg/Force3DoF': 'true',
        'Optimizer/Slam2D': 'true',
        # Build a 2D occupancy grid from the depth image so /map is directly
        # comparable with the slam_toolbox maps and usable by Nav2.
        #
        # NOTE: 'Grid/FromDepth' does NOT exist in rtabmap 0.22 - it was
        # replaced by 'Grid/Sensor' (0=scan, 1=depth, 2=both). Unknown names
        # are accepted silently, so passing the old one looks like it worked.
        # Check anything you set with:  ros2 param get /rtabmap Grid/Sensor
        'Grid/Sensor': '1',
        'Grid/3D': 'false',
        'Grid/CellSize': '0.05',
        'Grid/RangeMax': '5.0',
        # Ground/obstacle split by height above the floor. frame_id is
        # BuddyBaseFootprint_link, which sits ON the floor, so these are
        # absolute heights. The camera is at 0.864 m looking forward and
        # barely sees the floor at all (measured: only 3 of 17,743 projected
        # points fell below 0.08 m), so ground detection has little to work
        # with - which is fine, walls are what we want in the grid.
        'Grid/MaxGroundHeight': '0.10',
        'Grid/MaxObstacleHeight': '1.5',
        'Grid/NormalsSegmentation': 'false',
        # Fill the space between the camera and each obstacle as free, rather
        # than leaving it unknown. Without this the grid is isolated specks of
        # free space with no connected structure, which Nav2 cannot plan in.
        'Grid/RayTracing': 'true',
        # Grid/FootprintHeight is deliberately NOT set. It filters points over
        # the robot's own footprint and needs Grid/FootprintLength and
        # Grid/FootprintWidth set with it. Setting it to the sensor height
        # (0.9) on 2026-11-12 silently discarded the entire obstacle band and
        # produced a map with ZERO occupied cells - all free space, no walls.
        # Map update rate. The camera runs at 15 fps; processing every frame
        # saturates the Orin Nano for no mapping benefit.
        'Rtabmap/DetectionRate': '2.0',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'Mem/IncrementalMemory': 'true',
    }]

    remappings = [
        ('rgb/image', f'/{name}/rgb/image_raw'),
        ('rgb/camera_info', f'/{name}/rgb/camera_info'),
        ('depth/image', f'/{name}/stereo/image_raw'),
    ]

    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        parameters=rtabmap_params,
        remappings=remappings,
        # -d wipes the previous database so each run starts from a clean map.
        # Drop it (localization:=true) to reuse what was already built.
        arguments=['-d'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='false'),
        camera,
        rtabmap,
    ])
