#!/usr/bin/env python

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


import time
from typing import List
import unittest
from dataclasses import dataclass
import pytest
import rclpy
from rclpy.node import Node

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

    description_package: str
    description_macro_file: str
    robot_name: str
    controllers_file: str


def get_launch_arg_choices(package_name: str, launch_file_name: str) -> dict:
    launch_description_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_file_name])
    )
    launch_description = launch_description_source.try_get_launch_description_without_context()
    return {arg.name: arg.choices for arg in launch_description.get_launch_arguments()}


def get_possible_combinations(
    package_name: str,
    launch_file_name: str,
    robot_names_7dof: List[str],
    description_package_arg_name: str,
    description_macro_file_arg_name: str,
    controllers_file_arg_name: str,
) -> List[KukaArgCombination]:
    launch_arg_choices = get_launch_arg_choices(package_name, launch_file_name)
    description_package_choices = launch_arg_choices[description_package_arg_name]
    description_macro_file_choices = launch_arg_choices[description_macro_file_arg_name]
    controllers_file_choices = launch_arg_choices[controllers_file_arg_name]

    if len(controllers_file_choices) != 2:
        raise ValueError("this script accepts exactly 2 controller files")

    if "6dof" in controllers_file_choices[0]:
        controllers_file_6dof, controllers_file_7dof = controllers_file_choices
    else:
        controllers_file_7dof, controllers_file_6dof = controllers_file_choices

    possible_combinations = []
    for description_package in description_package_choices:
        package_prefix = description_package.split("_")[1]
        for description_macro_file in description_macro_file_choices:
            char_after_prefix = description_macro_file[len(package_prefix)]
            if (
                description_macro_file.startswith(package_prefix)
                and not char_after_prefix.isnumeric()
            ):
                robot_name = description_macro_file.split("_macro")[0]
                possible_combinations.append(
                    KukaArgCombination(
                        description_package=description_package,
                        description_macro_file=description_macro_file,
                        robot_name=robot_name,
                        controllers_file=controllers_file_7dof
                        if robot_name in robot_names_7dof
                        else controllers_file_6dof,
                    )
                )
    return possible_combinations


package_name = "kuka_ros2_control_support"
launch_file_name = "test_bringup.launch.py"
robot_names_7dof = ["lbr_iiwa_14_r820"]
description_package_arg_name = "description_package"
description_macro_file_arg_name = "description_macro_file"
controllers_file_arg_name = "controllers_file"
extra_launch_args = {"use_mock_hardware": "true", "rviz_file": "view_robot.rviz"}

possible_combinations = get_possible_combinations(
    package_name=package_name,
    launch_file_name=launch_file_name,
    robot_names_7dof=robot_names_7dof,
    description_package_arg_name=description_package_arg_name,
    description_macro_file_arg_name=description_macro_file_arg_name,
    controllers_file_arg_name=controllers_file_arg_name,
)

launch_testing_params = [
    (
        f"{description_package_arg_name}={combination.description_package} "
        f"{description_macro_file_arg_name}={combination.description_macro_file} "
        f"{controllers_file_arg_name}={combination.controllers_file}"
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

    global current_combination, possible_combinations, package_name, launch_file_name, extra_launch_args
    global description_package_arg_name, description_macro_file_arg_name, controllers_file_arg_name
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
            f"{description_package_arg_name}": launch_args[description_package_arg_name],
            f"{description_macro_file_arg_name}": launch_args[description_macro_file_arg_name],
            f"{controllers_file_arg_name}": launch_args[controllers_file_arg_name],
            **extra_launch_args,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [ReadyToTest(), robot_driver])


class RobotDriverTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node_name = "ros2_control_support_test"
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
