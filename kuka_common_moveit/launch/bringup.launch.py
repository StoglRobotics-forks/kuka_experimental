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

import os
import yaml
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro
from launch_ros.substitutions import FindPackageShare


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


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
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="kuka_common_moveit",
            description="Package with semantic robot description needed for MoveIt2.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "semantic_description_file",
            default_value="common_kuka.srdf.xacro",
            description="Semantic robot description file (SRDF/XACRO) with of the robot or \
            application. The expected location of the file is '<moveit_config_package>/srdf/'.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_file",
            default_value="moveit.rviz",
            description="Rviz2 configuration file of the visualization. \
            The expected location of the file is '<moveit_config_package>/rviz/'.",
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

    moveit_config_package = LaunchConfiguration("moveit_config_package")
    semantic_description_file = LaunchConfiguration("semantic_description_file")
    rviz_file = LaunchConfiguration("rviz_file")

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
        parameters=[robot_description]
    )

    # ros2_control_node
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(configuration_package), "config", controllers_file]
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Spawn controllers
    load_and_activate_controllers = []
    for controller in ["position_trajectory_controller", "joint_state_broadcaster"]:
        load_and_activate_controllers += [
            ExecuteProcess(
                cmd=[f"ros2 run controller_manager spawner {controller}"],
                shell=True,
                output="screen",
            )
        ]

    # MoveIt2 Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", semantic_description_file]
            ),
            " ",
            "robot_name:=",
            robot_name,
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    kinematics_yaml = load_yaml(
        "kuka_common_moveit", "config/kinematics.yaml"
    )

    move_group_config = {
        "planning_pipelines": ["ompl", "pilz"],
    }
    # Planning Functionality
    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    pilz_planning_pipeline_config = {
        "pilz": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "default_planner_config": "PTP"
        },
        #"pilz_lin": {
        #    "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
        #    "default_planner_config": "LIN",
        #}
    }
    ompl_planning_yaml = load_yaml(
        "kuka_common_moveit", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    pilz_planning_yaml = load_yaml(
        "kuka_common_moveit", "config/pilz_planning.yaml"
    )
    pilz_planning_pipeline_config["pilz"].update(pilz_planning_yaml)
    
    pilz_limits_yaml = load_yaml(
        "kuka_common_moveit", "config/pilz_cartesian_limits.yaml"
    )
    robot_description_planning_config = {
        "robot_description_planning" : pilz_limits_yaml
    }

    joint_limits_yaml = load_yaml(
        "kuka_common_moveit", "config/joint_limits.yaml"
    )
    robot_description_planning_config["robot_description_planning"].update(joint_limits_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "kuka_common_moveit", "config/kuka_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 100.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # MoveIt2
    moveit_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            move_group_config,
            ompl_planning_pipeline_config,
            pilz_planning_pipeline_config,
            robot_description_planning_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # Rviz2
    rviz_config = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", rviz_file])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            ros2_control_node,
            moveit_node,
            rviz_node,
        ]
        + load_and_activate_controllers
    )
