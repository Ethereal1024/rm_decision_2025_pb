from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    navigator_node = Node(
        package="navigator",
        executable="navigator_node",
        name="navigator",
        output="screen",
    )

    rotator_node = Node(
        package="rotator",
        executable="rotator_node",
        name="rotator",
        output="screen",
    )

    decision_node = Node(
        package="decision_test_alpha",
        executable="decision_test_alpha_node",
        name="decision_test_alpha",
        output="screen",
    )

    # 将所有节点添加到LaunchDescription
    ld = LaunchDescription()
    ld.add_action(navigator_node)
    ld.add_action(rotator_node)
    ld.add_action(decision_node)

    return ld
