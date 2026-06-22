from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    # Hanoi demo node
    hanoi_demo = Node(
        package="moveit_hanoi",
        executable="two_hanoi_node",
        output="screen",
        parameters=[
            moveit_config
        ]
    )

    return LaunchDescription([hanoi_demo])