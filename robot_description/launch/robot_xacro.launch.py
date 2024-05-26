import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

import xacro


def generate_launch_description():

    description_package_name = "robot_description"
    install_dir = get_package_prefix(description_package_name)
    print("isto é:  " + install_dir)


    robot_model_path = os.path.join(
        get_package_share_directory('robot_description'))

    xacro_file = os.path.join(robot_model_path, 'urdf', 'robot.xacro')

    # convert XACRO file into URDF
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(description_package_name), 'rviz', 'robot_urdf_vis.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])
    return LaunchDescription([
        robot_state_publisher,
        #rviz_node,
    ])