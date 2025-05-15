from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch.events import TimerEvent
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.event_handlers import OnProcessStart
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    ld = LaunchDescription()

    description_package = get_package_share_directory('palletizing_robot_description')
    urdf_file = os.path.join(description_package, 'urdf', 'palletizing_robot.urdf.xacro')
    controller_file = os.path.join(get_package_share_directory('palletizing_robot_gazebo'), 'config', 'controller.yaml')
    gazebo_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    world_file = os.path.join(get_package_share_directory('palletizing_robot_gazebo'), 'worlds', 'robot_environment.world')
    robot_description = Command(['xacro ', urdf_file])

    moveit_config = (
        MoveItConfigsBuilder("palletizing_robot", package_name="palletizing_robot_moveit_config")
        .robot_description(file_path="config/pr.urdf.xacro")
        .robot_description_semantic(file_path="config/pr.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    x_arg = DeclareLaunchArgument('x', default_value='0')
    y_arg = DeclareLaunchArgument('y', default_value='0')
    z_arg = DeclareLaunchArgument('z', default_value='0')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'debug': 'false',
            'gui': 'true',
            'world' : world_file,
        }.items()
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("palletizing_robot_moveit_config"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    spawn_the_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'pr',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen',
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[moveit_config.robot_description, controller_file, {"use_sim_time": True}],
        output='screen',
        remappings=[("~/robot_description", "/robot_description")]
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    use_sim_time={"use_sim_time": True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )
    delay_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_controller_spawner],
        )
    )
    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[rviz_node],
        )
    )
    delayed_move_group = TimerAction(
        period=5.0,
        actions=[move_group_node],
    )

    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(gazebo)
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_the_robot)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(delayed_move_group)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_arm_controller)
    ld.add_action(delay_rviz_node)

    return ld