import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'vslam_navigation'
    drl_pkg = get_package_share_directory('drl_navigation')

    # OmniWheelPlugin must be findable by Gazebo
    plugin_path = os.path.join(
        get_package_prefix('robot_description'), 'lib', 'robot_description')
    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=plugin_path + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', ''))

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(drl_pkg, 'worlds', 'training.world'),
        description='Full path to the world file to load')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set false for headless')

    world_file = LaunchConfiguration('world_file')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            drl_pkg, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items())

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml'),
            {'use_sim_time': True}])

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
            'gui':   LaunchConfiguration('gui'),
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items())

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05'],
        output='screen')

    return LaunchDescription([
        set_plugin_path,
        declare_world_file_cmd,
        declare_gui_cmd,
        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
    ])
