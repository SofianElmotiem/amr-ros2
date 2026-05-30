import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    il_pkg  = get_package_share_directory('imitation_learning')
    drl_pkg = get_package_share_directory('drl_navigation')

    # Point Gazebo to the omni_wheel plugin and to our custom models folder
    plugin_path = os.path.join(
        get_package_prefix('robot_description'), 'lib', 'robot_description')

    # models/ lives two levels above the install share directory
    models_path = os.path.normpath(
        os.path.join(get_package_prefix('imitation_learning'),
                     '..', '..', '..', 'src', 'models'))

    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=plugin_path + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', ''))

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''))

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set false for headless')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            drl_pkg, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items())

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[
            os.path.join(drl_pkg, 'config', 'twist_mux.yaml'),
            {'use_sim_time': True}])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': os.path.join(il_pkg, 'worlds', 'imitation_learning.world'),
            'gui':   LaunchConfiguration('gui'),
            'extra_gazebo_args': '--ros-args --params-file ' +
                                 os.path.join(drl_pkg, 'config', 'gazebo_params.yaml')
        }.items())

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot',
                   '-x', '-2.8', '-y', '2.8', '-z', '0.05'],
        output='screen')

    return LaunchDescription([
        set_plugin_path,
        set_model_path,
        declare_gui,
        rsp,
        twist_mux,
        gazebo,
        spawn,
    ])
