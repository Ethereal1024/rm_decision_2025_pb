from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    standard_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('standard_robot_pp_ros2'),
            'launch',
            'standard_robot_pp_ros2.launch.py'
        ))
    )

    pb2025_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('pb2025_nav_bringup'),
            'launch',
            'rm_navigation_reality_launch.py'
        )),
        launch_arguments={
            'world': 'test',
            'slam': 'False',
            'use_robot_state_pub': 'False'
        }.items()
    )

    # decision_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('decision_bringup'),
    #         'launch',
    #         'test_bringup.launch.py'
    #     ))
    # )

    ld = LaunchDescription()
    ld.add_action(standard_robot_launch)
    ld.add_action(pb2025_nav_launch)
    # ld.add_action(decision_launch)

    return ld
