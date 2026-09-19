"""Launch the RB3302 model in its idealized project course."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory("rb3302_sim")
    model_share = get_package_share_directory("robot_model")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_path = PathJoinSubstitution([sim_share, "worlds", "rb3302_course.sdf"])
    bridge_path = PathJoinSubstitution([sim_share, "config", "bridge.yaml"])
    model_path = PathJoinSubstitution([model_share, "urdf", "diffbot.urdf.xacro"])

    gz_args = LaunchConfiguration("gz_args")
    start_x = LaunchConfiguration("x")
    start_y = LaunchConfiguration("y")
    start_yaw = LaunchConfiguration("yaw")

    robot_description = ParameterValue(
        Command(["xacro ", model_path, " use_sim:=true"]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ros_gz_sim_share, "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [gz_args, " ", world_path]}.items(),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_rb3302",
        output="screen",
        arguments=[
            "-world", "rb3302_course",
            "-name", "diffbot",
            "-topic", "robot_description",
            "-x", start_x,
            "-y", start_y,
            "-z", "0.07",
            "-Y", start_yaw,
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_path}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gz_args",
            default_value="-r -v 3",
            description="Gazebo arguments; use '-r -s -v 3' for headless mode",
        ),
        DeclareLaunchArgument("x", default_value="0.30"),
        DeclareLaunchArgument("y", default_value="0.30"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        gazebo,
        state_publisher,
        spawn_robot,
        bridge,
    ])
