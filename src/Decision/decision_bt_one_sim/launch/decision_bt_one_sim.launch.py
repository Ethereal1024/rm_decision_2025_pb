import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('decision_bt_one_sim'), 'config', 'params.yaml')
    
    return LaunchDescription([
        Node(
            package='decision_bt_one_sim',
            executable='decision_bt_one_sim_node',
            name='decision_bt_one_sim_node',
            parameters=[params_file]
        )
    ])
