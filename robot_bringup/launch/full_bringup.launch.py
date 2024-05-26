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

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("puma2_description"), "launch"
                ),
                "/puma2_load_description.launch.py",
            ]
        ),
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("puma2_bringup"), "launch"),
                "/sensors.launch.py",
            ]
        ),
    )

    # Ros2 Control Configuration
    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_controller_traj = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    spawn_controller_velocity = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        output="screen",
    )

    swerve_controller = Node(
        package="puma2_simulation",
        executable="swerve_control.py",
        name="swerve_control",
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("puma2_localization"), "launch"
                ),
                "/puma2_localization.launch.py",
            ]
        ),
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("twist_mux"), "launch"),
                "/twist_mux_launch.py",
            ]
        ),
    )

    return LaunchDescription(
        [
            robot_description,
            sensors_launch,
            spawn_controller,
            spawn_controller_traj,
            spawn_controller_velocity,
            swerve_controller,
            localization_launch,
            twist_mux_launch,
        ]
    )
