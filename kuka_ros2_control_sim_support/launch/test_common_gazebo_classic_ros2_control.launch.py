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
#
#
# Author: Dr. Denis

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="kuka_resources",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="kuka_6dof_controllers.yaml",
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
                ],
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
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
            "robot_controller",
            default_value="forward_position_controller",
            choices=["forward_position_controller", "joint_trajectory_controller"],
            description="Robot controller to start.",
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
                ],
            description="NOTE:robot name and robot description macro name are same",
        )
    )
    
    
    

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_macro_file = LaunchConfiguration("robot_description_macro_file")
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    robot_controller = LaunchConfiguration("robot_controller")

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "view_robot.rviz"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kuka_ros2_control_sim_support"), "urdf", "common_sim_kuka.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "mock_sensor_commands:=false",
            " ",
            "sim_gazebo_classic:=true",
            " ",
            "sim_gazebo:=false",
            " ",
            "simulation_controllers:=",
            robot_controllers,
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
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # # Gazebo node
    gazebo_classic = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    # Spawn robot
    
    classic_spawn_robot = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=["-topic", "robot_description","-entity", "$ROBOT_NAME$"],
        output='screen',
        
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controllers = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=classic_spawn_robot,

            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    # Delay loading and activation of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[
                        TimerAction(
                            period=3.0,
                            actions=[controller],
                        ),
                    ],
                )
            )
        ]

    return LaunchDescription(
        declared_arguments
        + [
            gazebo_classic,
            classic_spawn_robot,
            robot_state_pub_node,
            rviz_node,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )
