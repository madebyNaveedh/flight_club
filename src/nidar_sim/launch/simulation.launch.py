#!/usr/bin/env python3
"""
NIDAR AirMouse Master Simulation & Autonomy Launch File
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nidar_sim = get_package_share_directory('nidar_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_sdf = os.path.join(pkg_nidar_sim, 'worlds', 'nidar_airmouse_arena.sdf')
    bridge_config = os.path.join(pkg_nidar_sim, 'launch', 'bridge_config.yaml')
    ref_grid_json = os.path.join(pkg_nidar_sim, 'worlds', 'reference_grid.json')

    # 1. Start Gazebo Sim with NIDAR World & Drone
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 2 {world_sdf}'}.items()
    )

    # 2. ROS-Gazebo Parameter Bridge (LaserScan, Odometry, Twist)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}]
    )

    # 3. High-Performance Zero-Copy Image Bridge for Camera
    gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
    )

    # 4. Survivor Perception & Grid Localiser (YOLOv8 + CUDA)
    survivor_detector = Node(
        package='nidar_perception',
        executable='survivor_detector',
        output='screen',
        parameters=[{
            'image_topic': '/camera/image_raw',
            'odom_topic': '/odom',
            'conf_threshold': 0.35,
            'max_survivors': 6,
            'arena_size': 15.0
        }]
    )

    # 5. Real-Time Scorer
    grid_scorer = Node(
        package='nidar_perception',
        executable='grid_scorer',
        output='screen',
        parameters=[{'reference_json': ref_grid_json}]
    )

    # 6. NIDAR GCS Web Server
    gcs_server = Node(
        package='nidar_gcs',
        executable='gcs_server',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        gz_bridge,
        gz_image_bridge,
        survivor_detector,
        grid_scorer,
        gcs_server
    ])
