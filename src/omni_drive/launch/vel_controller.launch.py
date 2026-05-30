from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    Node(
        package = "joy",
        executable = "joy_node"
        ),
    Node(
        package='omni_drive',
        executable='pub_vel.py',
        output='screen'
        ),
    ])