from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("omni_robot", package_name="omni_robot_moveit_config")
        .to_moveit_configs()
    )

    # Generate the standard MoveIt demo launch
    ld = generate_demo_launch(moveit_config)

    # Add a launch argument: use_sim_time
    ld.entities.insert(
        0, DeclareLaunchArgument("use_sim_time", default_value="true")
    )

    # Inject the use_sim_time param into all nodes that support it
    for entity in ld.entities:
        if hasattr(entity, "parameters") and isinstance(entity.parameters, list):
            entity.parameters.append({"use_sim_time": LaunchConfiguration("use_sim_time")})

    return ld
