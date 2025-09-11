from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # --- MoveIt config (όπως είχες) ---
    moveit_config = MoveItConfigsBuilder(
        "my_robot", package_name="my_robot_moveit_config"
    ).to_moveit_configs()

    # --- Args ---
    robot_name = LaunchConfiguration("robot_name")
    world_path = LaunchConfiguration("world")

    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="my_robot", description="Gazebo entity name"
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([
            FindPackageShare("my_robot_bringup"), "worlds", "world_ATHENA_FOR_ARUCO.world"
        ]),
        description="Path to Gazebo world file",
    )

    # --- Gazebo (classic) server+client ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world_path}.items(),
    )

    # --- Spawn the robot from the MoveIt-provided robot_description ---
    # Προσοχή: το MoveIt demo ήδη δημοσιεύει /robot_description.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_my_robot",
        arguments=[
            "-topic", "robot_description",
            "-entity", robot_name,
        ],
        output="screen",
    )

    # --- MoveIt demo (RViz + fake controllers κλπ) ---
    moveit_demo = generate_demo_launch(moveit_config)

    return LaunchDescription([
        declare_robot_name,
        declare_world,
        moveit_demo,
        gazebo_launch,
        spawn_entity,
    ])
