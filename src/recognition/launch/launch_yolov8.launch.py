from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='yolov8_recognition',
            executable='yolov8_ros2_pt.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='yolov8_recognition',
            executable='detection_3d.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='yolov8_recognition',
            executable='detection_to_costmap.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='yolov8_recognition',
            executable='grasp_from_detection.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])