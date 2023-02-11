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

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration 


def generate_launch_description():

    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_package",
            default_value = "kuka_kr3_support",
            description = "Description package with robot URDF/xacro files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_file",
            default_value = "kr3r540.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    
    
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")


    load_and_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                    [FindPackageShare('kuka_resources'), "launch"
                    ,'test_ros2_control_kuka_xxx.launch.py'
                ])
        ),
        launch_arguments={
            "robot_description_package": robot_description_package,
            "robot_description_file": robot_description_file,
        }.items(),
    )
    
    launch_files_to_include = [
        load_and_test
    ]
    
    return LaunchDescription( declared_arguments + launch_files_to_include)