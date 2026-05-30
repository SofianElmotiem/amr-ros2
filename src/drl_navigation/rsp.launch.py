import os
import re

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def resolve_package_uris(xml_string):
    def replace(match):
        try:
            pkg_path = get_package_share_directory(match.group(1))
            return f'file://{pkg_path}/{match.group(2)}'
        except Exception:
            return match.group(0)
    return re.sub(r'package://([^/]+)/(.*?)(?=["\'])', replace, xml_string)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = os.path.join(
        get_package_share_directory('robot_description'), 'robot', 'omni_robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = resolve_package_uris(robot_description_config.toxml())

    params = {'robot_description': robot_description_xml, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])
