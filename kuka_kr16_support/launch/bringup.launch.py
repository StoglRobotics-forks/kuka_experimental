import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_nodes_to_launch(context, *args, **kwargs):
    list_nodes = LaunchConfiguration("list_nodes").perform(context)
    nodes_to_launch = LaunchConfiguration("nodes_to_launch").perform(context)
    nodes_map = kwargs
    if list_nodes == "true":
        print(f"Possible nodes you can launch are:{nodes_map.keys()}")
        exit()

    nodes = []

    nodes_to_launch = nodes_to_launch.replace(" ", "").split(",")
    for elem in nodes_to_launch:
        if elem in nodes_map:
            print(f"appending:{elem}")
            nodes.append(nodes_map[elem])
        else:
            print(
                f"{elem} is not a valid node you can launch. valid choices are:{nodes_map.keys()}"
            )
            exit()

    return nodes


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "nodes_to_launch",
            default_value="cn, rsp, jsb, rcs, rviz, jtc",
            description="list of all nodes you want to launch.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "list_nodes",
            default_value="false",
            description="list all nodes that can be launched.",
            choices=["true", "false"],
        )
    )

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
            choices=["true", "false"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
            choices=["true", "false"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "listen_ip_address",
            default_value="172.20.19.101",
            description="The ip address on of your device on which is listend.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "listen_port",
            default_value="49152",
            description="The port on which is listend.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "control_node",
            default_value="ros2_control_node_max_update_rate",
            description="Change the control node which is used.",
            choices=[
                "ros2_control_node",
                "ros2_control_node_max_update_rate",
                "ros2_control_node_fixed_period",
                "ros2_control_node_max_update_rate_sc",
                "ros2_control_node_fixed_period_sc",
            ],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Set the logging level of the loggers of all started nodes.",
            choices=[
                "debug",
                "DEBUG",
                "info",
                "INFO",
                "warn",
                "WARN",
                "error",
                "ERROR",
                "fatal",
                "FATAL",
            ],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level_all",
            default_value="info",
            description="Set the logging level of the loggers of all started nodes.",
            choices=[
                "debug",
                "DEBUG",
                "info",
                "INFO",
                "warn",
                "WARN",
                "error",
                "ERROR",
                "fatal",
                "FATAL",
            ],
        )
    )

    # initialize arguments
    prefix = LaunchConfiguration("prefix")
    use_mock_hw = LaunchConfiguration("use_mock_hw")
    use_mock_sensor_commands = LaunchConfiguration("use_mock_sensor_commands")
    listen_ip_address = LaunchConfiguration("listen_ip_address")
    listen_port = LaunchConfiguration("listen_port")
    control_node = LaunchConfiguration("control_node")
    log_level = LaunchConfiguration("log_level")
    log_level_all = LaunchConfiguration("log_level_all")

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
            "listen_ip_address:=",
            listen_ip_address,
            " ",
            "listen_port:=",
            listen_port,
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
        executable=control_node,
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            ["KukaSystemPositionOnlyHardware:=", log_level],
            "--ros-args",
            "--log-level",
            log_level_all,
        ],
        parameters=[robot_description, robot_controllers],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_trajectory_controller",
            "-c",
            "/controller_manager",
        ],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kuka_kr16_support"), "config/rviz", "view_robot_kr16_2.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("kuka_kr16_support"),
            "config/controller",
            "kuka_6dof_joint_trajectory_controller_goals.yaml",
        ]
    )

    joint_trajecotory_controller = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_joint_trajectory_controller",
        name="publisher_joint_trajectory_controller",
        parameters=[position_goals],
        output="both",
    )

    delay_joint_trajecotory_controller_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajecotory_controller],
            )
        )
    )

    nodes = {
        "cn": control_node,
        "rsp": robot_state_pub_node,
        "jsb": joint_state_broadcaster_spawner,
        "rcs": delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        "rviz": delay_rviz_after_joint_state_broadcaster_spawner,
        "jtc_after_jsb": delay_joint_trajecotory_controller_after_joint_state_broadcaster_spawner,
        "jtc": joint_trajecotory_controller,
    }

    return LaunchDescription(
        declared_arguments
        + [OpaqueFunction(function=get_nodes_to_launch, kwargs=nodes)]
    )
