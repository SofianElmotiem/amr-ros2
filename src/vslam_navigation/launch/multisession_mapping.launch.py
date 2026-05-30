import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_soprano = get_package_share_directory('vslam_navigation')
    
    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    rtabmap_params_file_arg = DeclareLaunchArgument(
        'rtabmap_params_file',
        default_value=os.path.join(pkg_soprano, 'config', 'rtabmap_params.yaml'),
        description='Full path to RTAB-Map parameters file'
    )
    
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap_multisession.db',
        description='Database path for multisession mapping'
    )
    
    new_session_arg = DeclareLaunchArgument(
        'new_session',
        default_value='false',
        description='Start a new mapping session (deletes previous map). Set false to continue previous session.'
    )
    
    rtabmap_viz_arg = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='true',
        description='Launch RTAB-Map visualization'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_soprano, 'config', 'soprano.rviz'),
        description='RViz configuration file'
    )
    
    compressed_arg = DeclareLaunchArgument(
        'compressed',
        default_value='false',
        description='Use compressed image transport (for bandwidth saving)'
    )
    
    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file')
    database_path = LaunchConfiguration('database_path')
    new_session = LaunchConfiguration('new_session')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    compressed = LaunchConfiguration('compressed')
    
    # Set use_sim_time globally
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # RGBD Sync Node (for uncompressed images)
    rgbd_sync_node = Node(
        condition=UnlessCondition(compressed),
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[rtabmap_params_file],
        remappings=[
            ('rgb/image', '/rgbd_camera/image_raw'),
            ('depth/image', '/rgbd_camera/depth/image_raw'),
            ('rgb/camera_info', '/rgbd_camera/camera_info'),
            ('rgbd_image', '/rgbd_image')
        ]
    )
    
    # RGBD Sync Node (for compressed images)
    rgbd_sync_compressed_node = Node(
        condition=IfCondition(compressed),
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[rtabmap_params_file,
                   {'rgb_image_transport': 'compressed',
                    'depth_image_transport': 'compressedDepth',
                    'approx_sync_max_interval': 0.02}],
        remappings=[
            ('rgb/image', '/rgbd_camera/image_raw'),
            ('depth/image', '/rgbd_camera/depth/image_raw'),
            ('rgb/camera_info', '/rgbd_camera/camera_info'),
            ('rgbd_image', '/rgbd_image')
        ]
    )
    
    # RTAB-Map SLAM Node (New Session - Deletes Database)
    rtabmap_new_session_node = Node(
        condition=IfCondition(new_session),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params_file,
                   {'database_path': database_path,
                    'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image', '/rgbd_camera/image_raw'),
            ('depth/image', '/rgbd_camera/depth/image_raw'),
            ('rgb/camera_info', '/rgbd_camera/camera_info'),
            ('scan', '/scan'),
            ('odom', '/odom')
        ],
        arguments=['-d']  # Delete database for new session
    )
    
    # RTAB-Map SLAM Node (Continue Session - Preserves Database)
    rtabmap_continue_session_node = Node(
        condition=UnlessCondition(new_session),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params_file,
                   {'database_path': database_path,
                    'use_sim_time': use_sim_time}],
        remappings=[
            ('rgb/image', '/rgbd_camera/image_raw'),
            ('depth/image', '/rgbd_camera/depth/image_raw'),
            ('rgb/camera_info', '/rgbd_camera/camera_info'),
            ('scan', '/scan'),
            ('odom', '/odom')
        ]
        # No '-d' argument - preserves database from previous session
    )
    
    # RTAB-Map Visualization
    rtabmap_viz_node = Node(
        condition=IfCondition(rtabmap_viz),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[rtabmap_params_file],
        remappings=[
            ('rgb/image', '/rgbd_camera/image_raw'),
            ('depth/image', '/rgbd_camera/depth/image_raw'),
            ('rgb/camera_info', '/rgbd_camera/camera_info'),
            ('scan', '/scan'),
            ('odom', '/odom')
        ]
    )
    
    # RViz Node
    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        rtabmap_params_file_arg,
        database_path_arg,
        new_session_arg,
        rtabmap_viz_arg,
        rviz_arg,
        rviz_config_arg,
        compressed_arg,
        
        # Set parameter
        set_use_sim_time,
        
        # Nodes
        rgbd_sync_node,
        rgbd_sync_compressed_node,
        rtabmap_new_session_node,
        rtabmap_continue_session_node,
        rtabmap_viz_node,
        rviz_node
    ])
