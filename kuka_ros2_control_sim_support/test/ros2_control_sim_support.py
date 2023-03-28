#!/usr/bin/env python
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


import time
import unittest
from dataclasses import dataclass
import pytest
import rclpy
from rclpy.node import Node
import subprocess

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
from launch_testing.actions import ReadyToTest


@dataclass
class KukaArgCombination:
    """Class for keeping track of arguments for testing ros2 control support package."""

    robot_description_package: str
    robot_description_macro_file: str
    robot_name: str
    controllers_file: str


def get_possible_combinations(get_combinations_cmd, robot_names_7dof=[]):
    def arg_str_to_values_array(arg_str):
        values = arg_str.split("[")[1].split("]")[0].split(",")
        return [v.strip().strip("'") for v in values]

    # Get available options for launch file
    process = subprocess.Popen(get_combinations_cmd, stdout=subprocess.PIPE)
    stdout, stderr = process.communicate()
    args_str = stdout.decode("utf-8").split("\n")[1:-1]

    # Extract available options for controllers_file, robot_description_package, robot_description_macro_file, and robot_name
    controllers_files = []
    robot_description_packages = []
    robot_description_macro_files = []
    robot_names = []
    for i, arg in enumerate(args_str):
        if "controllers_file" in arg:
            controllers_files = arg_str_to_values_array(args_str[i + 1])
        elif "robot_description_package" in arg:
            robot_description_packages = arg_str_to_values_array(args_str[i + 1])
        elif "robot_description_macro_file" in arg:
            robot_description_macro_files = arg_str_to_values_array(args_str[i + 1])
        elif "robot_name" in arg:
            robot_names = arg_str_to_values_array(args_str[i + 1])

    if len(controllers_files) != 2:
        raise ValueError("this script accepts exactly 2 contorller files")

    if "6dof" in controllers_files[0]:
        controller_file_6dof = controllers_files[0]
        controller_file_7dof = controllers_files[1]
    else:
        controller_file_6dof = controllers_files[1]
        controller_file_7dof = controllers_files[0]

    # Create all possible combinations
    possible_combinations = []
    for robot_description_package in robot_description_packages:
        package_prefix = robot_description_package.split("_")[1]
        for macro_file in robot_description_macro_files:
            char_after_prefix = macro_file[len(package_prefix)]
            if macro_file.startswith(package_prefix) and not char_after_prefix.isnumeric():
                robot_name = "kuka_" + macro_file.split("_macro")[0]
                if robot_name in robot_names:
                    arg_combination = KukaArgCombination(
                        robot_description_package=robot_description_package,
                        robot_description_macro_file=macro_file,
                        robot_name=robot_name,
                        controllers_file=controller_file_7dof
                        if robot_name in robot_names_7dof
                        else controller_file_6dof,
                    )
                    possible_combinations.append(arg_combination)

    return possible_combinations


robot_names_7dof = ["kuka_lbr_iiwa_14_r820"]
package_name = "kuka_ros2_control_sim_support"
launch_file_name = "test_common_gazebo_classic_ros2_control.launch.py"
get_combinations_cmd = ["ros2", "launch", package_name, launch_file_name, "--show-args"]
possible_combinations = get_possible_combinations(
    get_combinations_cmd, robot_names_7dof=robot_names_7dof
)

launch_testing_params = [
    (
        f"robot_description_package={combination.robot_description_package} "
        f"robot_description_macro_file={combination.robot_description_macro_file} "
        f"robot_name={combination.robot_name} "
        f"controllers_file={combination.controllers_file}"
    )
    for combination in possible_combinations
]

current_combination = -1


@pytest.mark.launch_test
@launch_testing.parametrize("launch_testing_params", launch_testing_params)
def generate_test_description(launch_testing_params):
    def parse_args():
        launch_args = {}
        for arg in launch_testing_params.split(" "):
            key, value = arg.split("=")
            launch_args[key] = value
        return launch_args

    global current_combination, possible_combinations, package_name, launch_file_name
    current_combination += 1

    print("-" * 20)
    print(f"Testing {current_combination+1}/{len(possible_combinations)} {launch_testing_params}")
    print("-" * 20)

    declared_arguments = []

    launch_args = parse_args()

    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_file_name])
        ),
        launch_arguments={
            "robot_description_package": launch_args["robot_description_package"],
            "robot_description_macro_file": launch_args["robot_description_macro_file"],
            "robot_name": launch_args["robot_name"],
            "controllers_file": launch_args["controllers_file"],
        }.items(),
    )

    return LaunchDescription(declared_arguments + [ReadyToTest(), robot_driver])


class RobotDriverTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node_name = "ros2_control_sim_support_test"
        cls.node = Node(cls.node_name)
        time.sleep(1)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    #
    # Test functions
    #

    def test_node_started(self):
        # Verify that the node started successfully
        node_names = self.node.get_node_names()
        self.assertIn(self.node_name, node_names)
