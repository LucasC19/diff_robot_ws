from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gnss_compass_node = Node(
        package="ros2-driver",
        executable="adnav_driver",
    )

    return LaunchDescription([gnss_compass_node])
