import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_soprano = get_package_share_directory('vslam_navigation')
    pkg_stereo_image_proc = get_package_share_directory('stereo_image_proc')

    # Paths
    stereo_image_proc_launch = PathJoinSubstitution(
        [pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py'])
    
    config_rviz = os.path.join(pkg_soprano, 'config', 'soprano.rviz')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    rviz = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    database_path = LaunchConfiguration('database_path')

    # RTAB-Map parameters for stereo
    parameters = {
        'frame_id': 'base_footprint',
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'approx_sync': True,  # CHANGED TO TRUE
        'subscribe_odom_info': True,
        'use_sim_time': use_sim_time,
        # Visual odometry parameters
        'OdomF2M/MaxSize': '1000',
        'GFTT/MinDistance': '10',
        'GFTT/QualityLevel': '0.00001',
        # SLAM parameters
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'Reg/Strategy': '0',  # CHANGED TO 0 for visual (stereo needs visual, not ICP)
        'Reg/Force3DoF': 'true',
        'Optimizer/GravitySigma': '0',
        'Grid/Sensor': '2',
        'Grid/RangeMax': '10.0',
        'Icp/CorrespondenceRatio': '0.2',
        'Icp/MaxCorrespondenceDistance': '0.15',
        'Icp/VoxelSize': '0.05',
    }

    remappings = [
        ('rgbd_image', '/stereo_camera/rgbd_image'),
        ('odom', '/vo'),
        ('scan', '/scan')
    ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Launch RTAB-Map UI'),
        
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'),
        
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Launch in localization mode'),
        
        DeclareLaunchArgument(
            'rviz_cfg',
            default_value=config_rviz,
            description='RViz configuration file'),
        
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap_stereo.db',
            description='Database path'),

        SetParameter(name='use_sim_time', value=use_sim_time),

        # RELAY NODES - Bridge Gazebo topics to stereo_image_proc expected topics
        Node(
            package='image_transport',
            executable='republish',
            name='relay_left_image',
            arguments=['raw', 'raw'],
            remappings=[
                ('in', '/left_camera/left_camera/image_raw'),  # FIXED
                ('out', '/stereo_camera/left/image_raw')
            ]
        ),

        Node(
            package='image_transport',
            executable='republish',
            name='relay_right_image',
            arguments=['raw', 'raw'],
            remappings=[
                ('in', '/right_camera/right_camera/image_raw'),  # FIXED
                ('out', '/stereo_camera/right/image_raw')
            ]
        ),

        Node(
            package='topic_tools',
            executable='relay',
            name='relay_left_camera_info',
            arguments=['/left_camera/left_camera/camera_info', '/stereo_camera/left/camera_info']  # FIXED
        ),

        Node(
            package='topic_tools',
            executable='relay',
            name='relay_right_camera_info',
            arguments=['/right_camera/right_camera/camera_info', '/stereo_camera/right/camera_info']  # FIXED
        ),
        # Stereo Image Processing
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([stereo_image_proc_launch]),
                    launch_arguments=[
                        ('left_namespace', 'stereo_camera/left'),
                        ('right_namespace', 'stereo_camera/right'),
                        ('disparity_range', '128'),
                    ]
                ),
            ]
        ),
        
        # Synchronize stereo data into single RGBD topic
        Node(
            package='rtabmap_sync',
            executable='stereo_sync',
            output='screen',
            namespace='stereo_camera',
            parameters=[{
                'use_sim_time': use_sim_time,
                'approx_sync': True  # ADDED
            }],
            remappings=[
                ('left/image_rect', 'left/image_rect'),
                ('right/image_rect', 'right/image_rect'),
                ('left/camera_info', 'left/camera_info'),
                ('right/camera_info', 'right/camera_info')
            ]
        ),

        # Visual Odometry from Stereo
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[parameters],
            remappings=remappings
        ),
        
        # SLAM Mode
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[parameters,
                       {'database_path': database_path}],
            remappings=remappings,
            arguments=['-d']
        ),
            
        # Localization Mode
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[parameters,
                       {'database_path': database_path,
                        'Mem/IncrementalMemory': 'False',
                        'Mem/InitWMWithAllNodes': 'True'}],
            remappings=remappings
        ),

        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            condition=IfCondition(rtabmap_viz),
            parameters=[parameters],
            remappings=remappings
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name="rviz2",
            output='screen',
            condition=IfCondition(rviz),
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_cfg]
        ),
    ])