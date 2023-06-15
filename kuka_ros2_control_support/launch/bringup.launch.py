# Software License Agreement (BSD License)
#
#  Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Univ of CO, Boulder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
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
            "robot_name",
            default_value="kuka",
            description="Name of the robot or application for unique identification.",
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
            "description_package",
            description="Description package with robot URDF/xacro files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_macro_file",
            description="URDF/XACRO description file with of the robot or application. \
            The expected location of the file is '<description_package>/urdf/'.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "configuration_package",
            default_value="kuka_resources",
            description="Package with configuration files, e.g., controllers, initial positions.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="kuka_6dof_controllers.yaml",
            description="YAML file with the controllers configuration. \
            The expected location of the file is '<configuration_package>/config/'.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            description="YAML file with the initial positions when using mock hardware from \
            ros2_control. The expected location of the file is '<configuration_package>/config/'.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_eki_communication",
            default_value="false",
            description="Use EKI communication to the KUKA Robot Controller (KR-C). \
            If the flag is set to true 'eki_robot_ip' and 'eki_robot_port' arguments define the \
            endpoint of robot controller to connect.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "eki_robot_ip",
            default_value="127.0.0.1",
            description="IP address for the robot controller for communication using EKI protocol.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "eki_robot_port",
            default_value="54600",
            description="Port by which the robot can be reached for communication using EKI protocol.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rsi_communication",
            default_value="false",
            description="Use RSI communication to the KUKA Robot Controller (KR-C). \
            If the flag is set to true 'rsi_listen_ip' and 'rsi_robot_port' arguments define the \
            endpoint where robot controller should connect to. (Keep in mind that you have to \
            define this endpoint also in the RSI configuration of the program.)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rsi_listen_ip",
            default_value="127.0.0.1",
            description="IP address of a computer where robot controller should connect for \
            communication using RSI protocol.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rsi_listen_port",
            default_value="49152",
            description="Port on a computer where robot controller should connect for \
            communication using RSI protocol.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_file",
            default_value="view_robot.rviz",
            description="Rviz2 configuration file of the visualization. \
            The expected location of the file is '<configuration_package>/config/'.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start vizualization in Rviz2.",
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
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    description_package = LaunchConfiguration("description_package")
    description_macro_file = LaunchConfiguration("description_macro_file")
    configuration_package = LaunchConfiguration("configuration_package")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")

    use_eki_communication = LaunchConfiguration("use_eki_communication")
    eki_robot_ip = LaunchConfiguration("eki_robot_ip")
    eki_robot_port = LaunchConfiguration("eki_robot_port")
    use_rsi_communication = LaunchConfiguration("use_rsi_communication")
    rsi_listen_ip = LaunchConfiguration("rsi_listen_ip")
    rsi_listen_port = LaunchConfiguration("rsi_listen_port")

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    rviz_file = LaunchConfiguration("rviz_file")
    start_rviz = LaunchConfiguration("start_rviz")

    log_level_driver = LaunchConfiguration("log_level_driver")
    log_level_all = LaunchConfiguration("log_level_all")

    initial_positions_file = PathJoinSubstitution(
        [FindPackageShare(configuration_package), "config", initial_positions_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kuka_ros2_control_support"), "urdf", "common_kuka.xacro"]
            ),
            " ",
            "robot_name:=",
            robot_name,
            " ",
            "prefix:=",
            prefix,
            " ",
            "description_package:=",
            description_package,
            " ",
            "description_macro_file:=",
            description_macro_file,
            " ",
            "use_eki_communication:=",
            use_eki_communication,
            " ",
            "eki_robot_ip:=",
            eki_robot_ip,
            " ",
            "eki_robot_port:=",
            eki_robot_port,
            " ",
            "use_rsi_communication:=",
            use_rsi_communication,
            " ",
            "rsi_listen_ip:=",
            rsi_listen_ip,
            " ",
            "rsi_listen_port:=",
            rsi_listen_port,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "initial_positions_file:=",
            initial_positions_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Publish TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_node",
        output="both",
        parameters=[robot_description],
    )

    # ros2_control_node
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(configuration_package), "config", controllers_file]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",  # optionally use `ros2_control_node` from ros2_control
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
        [FindPackageShare(configuration_package), "config", rviz_file]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
