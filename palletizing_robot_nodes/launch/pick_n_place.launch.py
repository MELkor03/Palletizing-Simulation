from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    spawner_node = Node(
        package='palletizing_robot_nodes',
        executable='spawner',
        name='spawner_node',
    )

    pick_place_node = Node(
        package='palletizing_robot_nodes',
        executable='pick_n_place',
        name='pick_n_place_node',
    )

    ld.add_action(spawner_node)
    ld.add_action(pick_place_node)

    return ld