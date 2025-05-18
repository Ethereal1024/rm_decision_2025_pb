from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav_bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    standard_robot_dir = get_package_share_directory("standard_robot_pp_ros2")

    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value="test",
        description="World name for map path construction",
    )

    standard_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    standard_robot_dir, "launch", "standard_robot_pp_ros2.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
    )

    pb2025_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(nav_bringup_dir, "launch", "rm_navigation_reality_launch.py")]
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "slam": "False",
            "use_robot_state_pub": "False",
            "use_sim_time": "False",
            "params_file": PathJoinSubstitution(
                [
                    nav_bringup_dir,
                    "config/reality/nav2_params.yaml",
                ]
            ),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_arg)
    ld.add_action(standard_robot_launch)
    ld.add_action(pb2025_nav_launch)

    return ld
