# Copyright 2023 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
            "controllers_file",
            default_value="kuka_6dof_controllers.yaml",
            choices=[
                "kuka_6dof_controllers.yaml",
                "kuka_7dof_controllers.yaml",
                # Note: for the robot kuka_lbr_iiwa_14_r820, kuka_7dof_controllers.yaml should be used
                # and the rest use kuka_6dof_controllers.yaml
            ],
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_package",
            choices=[
                "kuka_kr3_support",
                "kuka_kr5_support",
                "kuka_kr6_support",
                "kuka_kr10_support",
                "kuka_kr16_support",
                "kuka_kr120_support",
                "kuka_kr150_support",
                "kuka_kr210_support",
                "kuka_lbr_iiwa_support",
            ],
            description="Description package with robot URDF/xacro files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_macro_file",
            choices=[
                "kr3r540_macro.xacro",
                "kr5_arc_macro.xacro",
                "kr6r700sixx_macro.xacro",
                "kr6r900_2_macro.xacro",
                "kr6r900sixx_macro.xacro",
                "kr10r900_2_macro.xacro",
                "kr10r1100sixx_macro.xacro",
                "kr10r1420_macro.xacro",
                "kr16_2_macro.xacro",
                "kr120r2500pro_macro.xacro",
                "kr150_2_macro.xacro",
                "kr150r3100_2_macro.xacro",
                "kr210l150_macro.xacro",
                "lbr_iiwa_14_r820_macro.xacro",
            ],
            description="URDF/XACRO description file with the robot.",
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
            "use_mock_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            choices=[
                "kuka_kr3r540",
                "kuka_kr5_arc",
                "kuka_kr6r700sixx",
                "kuka_kr6r900_2",
                "kuka_kr6r900sixx",
                "kuka_kr10r900_2",
                "kuka_kr10r1100sixx",
                "kuka_kr10r1420",
                "kuka_kr16_2",
                "kuka_kr120r2500pro",
                "kuka_kr150_2",
                "kuka_kr150r3100_2",
                "kuka_kr210l150",
                "kuka_lbr_iiwa_14_r820",
            ],
            description="NOTE:robot name and robot description macro name are same",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "listen_ip_address",
            default_value="127.0.0.1",
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
            "log_level_driver",
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
    controllers_file = LaunchConfiguration("controllers_file")
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_macro_file = LaunchConfiguration("robot_description_macro_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_name = LaunchConfiguration("robot_name")
    listen_ip_address = LaunchConfiguration("listen_ip_address")
    listen_port = LaunchConfiguration("listen_port")
    log_level_driver = LaunchConfiguration("log_level_driver")
    log_level_all = LaunchConfiguration("log_level_all")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_ros2_control_support"),
                    "urdf",
                    "common_kuka.xacro",
                ]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "controllers_file:=",
            controllers_file,
            " ",
            "robot_description_package:=",
            robot_description_package,
            " ",
            "robot_description_macro_file:=",
            robot_description_macro_file,
            " ",
            "robot_name:=",
            robot_name,
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
        [FindPackageShare("kuka_resources"), "config", controllers_file]
    )

    control_node = Node(
        package="kuka_ros2_control_support",
        executable="ros2_control_node_steady_clock",
        output="both",
        arguments=[
            "--ros-args",
            "--log-level",
            ["KukaSystemPositionOnlyHardware:=", log_level_driver],
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
        arguments=["position_trajectory_controller", "-c", "/controller_manager"],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kuka_resources"), "config", "view_robot.rviz"]
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

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
