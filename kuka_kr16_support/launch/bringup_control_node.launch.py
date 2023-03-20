import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers' configuration \
            have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hw",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )

    # initialize arguments
    prefix = LaunchConfiguration("prefix")
    use_mock_hw = LaunchConfiguration("use_mock_hw")
    use_mock_sensor_commands = LaunchConfiguration("use_mock_sensor_commands")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kuka_kr16_support"), "urdf", "kr16_2.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hw:=",
            use_mock_hw,
            " ",
            "use_mock_sensor_commands:=",
            use_mock_sensor_commands,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("common_6dof_position_only_system"),
            "config",
            "kuka_6dof_controller_position.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node_max_update_rate",
        output="both",
        parameters=[robot_description, robot_controllers],
    )

    nodes = [
        control_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
