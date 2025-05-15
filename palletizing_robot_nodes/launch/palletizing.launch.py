from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    spawner_node = Node(
        package='palletizing_robot_nodes',
        executable='spawner',
        name='spawner_node',
        parameters=[{'use_sim_time': True}],
    )

    logger_node = Node(
        package='palletizing_robot_nodes',
        executable='end_effector_logger_node',
        name='end_effector_logger_node',
        parameters=[{'use_sim_time': True}],
    )

    palletization_service = Node(
        package='palletizing_robot_nodes',
        executable='palletization_service',
        name='palletization_service_node',
        parameters=[{'use_sim_time': True}],
    )

    delayed_actions = TimerAction(
        period=3.0,
        actions=[spawner_node, logger_node]
    )

    ld.add_action(palletization_service)
    ld.add_action(delayed_actions)

    return ld