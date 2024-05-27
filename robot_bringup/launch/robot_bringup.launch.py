#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    foxglove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("foxglove_bridge"), "launch"),
                "/foxglove_bridge_launch.xml",
            ]
        ),
    ) 

    # Ros2 Control Configuration
    motor_control_launch = Node(
        package="motor_control",
        executable="motor_control",
        output="screen",
    )

    odometry_launch = Node(
        package="motor_control",
        executable="odometry",
        output="screen",
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("twist_mux"), "launch"),
                "/twist_mux_launch.py",
            ]
        ),
    ) 

    pkg_name = 'robot_description'
    file_subpath = 'urdf/robot.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    robot_state_publisher_launch = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': False}]
    )

    return LaunchDescription(
        [
            foxglove_launch,
            motor_control_launch,
            odometry_launch,
            twist_mux_launch,
            robot_state_publisher_launch,
        ]
    )
