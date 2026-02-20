#!/usr/bin/env python3
"""
Launch file untuk TurtleBot3 Square Mission (C++) - ROS 2
Menjalankan Gazebo + Square Mission C++ node secara bersamaan.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Arguments
    model_arg = DeclareLaunchArgument(
        'model', default_value='burger',
        description='TurtleBot3 model (burger, waffle, waffle_pi)'
    )
    side_length_arg = DeclareLaunchArgument(
        'side_length', default_value='2.0',
        description='Panjang sisi persegi (meter)'
    )
    threshold_arg = DeclareLaunchArgument(
        'threshold', default_value='0.3',
        description='Threshold jarak waypoint (meter)'
    )

    # Include TurtleBot3 Gazebo World
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Square Mission Node (C++)
    square_mission_node = Node(
        package='turtlebot3_square_mission_cpp',
        executable='square_mission',
        name='square_mission',
        output='screen',
        parameters=[{
            'side_length': LaunchConfiguration('side_length'),
            'threshold': LaunchConfiguration('threshold'),
            'linear_speed': 0.2,
            'angular_speed': 0.5,
            'angle_threshold': 0.05,
            'rate_hz': 20,
        }]
    )

    return LaunchDescription([
        model_arg,
        side_length_arg,
        threshold_arg,
        gazebo_launch,
        square_mission_node,
    ])
