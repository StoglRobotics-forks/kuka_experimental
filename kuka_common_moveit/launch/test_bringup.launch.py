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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            ],
            description="YAML file with the controllers configuration. Only LBR IIWA 14 has 7 dofs.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            choices=[
                "initial_positions.yaml",
                "initial_positions_all_zeros.yaml",
            ],
            description="YAML file with the initial positions when using mock hardware from \
            ros2_control. Robots using '_all_zeros' file are: \
              - kr210l150 \
              - "
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            description="Description package with robot URDF/xacro files.",
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
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_macro_file",
            description="URDF/XACRO description file with of the robot or application. \
            The expected location of the file is '<description_package>/urdf/'.",
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
                "kr210r3100_macro.xacro",
                "lbr_iiwa_14_r820_macro.xacro",
            ],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_eki_communication",
            default_value="false",
            description="Use EKI communication to the KUKA Robot Controller (KR-C). If the flag is set to true 'eki_robot_ip' and 'eki_robot_port' arguments define the endpoint of robot controller to connect.",
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
            description="Use RSI communication to the KUKA Robot Controller (KR-C). If the flag is set to true 'rsi_listen_ip' and 'rsi_robot_port' arguments define the endpoint where robot controller should connect to. (Keep in mind that you have to define this endpoint also in the RSI configuration of the program.)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rsi_listen_ip",
            default_value="127.0.0.1",
            description="IP address of a computer where robot controller should connect for communication using RSI protocol.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rsi_listen_port",
            default_value="49152",
            description="Port on a computer where robot controller should connect for communication using RSI protocol.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    # initialize arguments
    controllers_file = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    description_package = LaunchConfiguration("description_package")
    description_macro_file = LaunchConfiguration("description_macro_file")

    use_eki_communication = LaunchConfiguration("use_eki_communication")
    eki_robot_ip = LaunchConfiguration("eki_robot_ip")
    eki_robot_port = LaunchConfiguration("eki_robot_port")
    use_rsi_communication = LaunchConfiguration("use_rsi_communication")
    rsi_listen_ip = LaunchConfiguration("rsi_listen_ip")
    rsi_listen_port = LaunchConfiguration("rsi_listen_port")

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    moveit_setup_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kuka_common_moveit'), 'launch', 'bringup.launch.py'])),
        launch_arguments={
            "robot_name": "kuka_test_moveit_setup",
            "controllers_file": controllers_file,
            "initial_positions_file": initial_positions_file,
            "description_package": description_package,
            "description_macro_file": description_macro_file,
            "use_eki_communication": use_eki_communication,
            "eki_robot_ip": eki_robot_ip,
            "eki_robot_port": eki_robot_port,
            "use_rsi_communication": use_rsi_communication,
            "rsi_listen_ip": rsi_listen_ip,
            "rsi_listen_port": rsi_listen_port,
            "use_mock_hardware": use_mock_hardware,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [moveit_setup_bringup_launch])
