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
import unittest
from dataclasses import dataclass
from typing import List

import launch_testing
import pytest
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass
class KukaArgCombination:
    """Class for keeping track of arguments for testing ros2 control support package."""

    description_package: str
    description_macro_file: str
    robot_name: str
    controllers_file: str
    init_pos_file: str


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
    initial_positions_file_arg_name: str,
    robot_names_init_pos_all_zeros: str,
) -> List[KukaArgCombination]:
    launch_arg_choices = get_launch_arg_choices(package_name, launch_file_name)
    description_package_choices = launch_arg_choices[description_package_arg_name]
    description_macro_file_choices = launch_arg_choices[description_macro_file_arg_name]
    controllers_file_choices = launch_arg_choices[controllers_file_arg_name]
    initial_positions_file_choices = launch_arg_choices[initial_positions_file_arg_name]

    if len(controllers_file_choices) != 2:
        raise ValueError("this script accepts exactly 2 controller files")

    if "6dof" in controllers_file_choices[0]:
        controllers_file_6dof, controllers_file_7dof = controllers_file_choices
    else:
        controllers_file_7dof, controllers_file_6dof = controllers_file_choices

    if len(initial_positions_file_choices) != 2:
        raise ValueError("this script accepts exactly 2 initial positions files")

    if "all_zeros" in initial_positions_file_choices[0]:
        init_pos_all_zeros, init_pos_default = initial_positions_file_choices
    else:
        init_pos_default, init_pos_all_zeros = initial_positions_file_choices

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
                        init_pos_file=init_pos_all_zeros
                        if robot_name in robot_names_init_pos_all_zeros
                        else init_pos_default,
                    )
                )
    return possible_combinations


### For test debugging ###
test_only_these_robot_names = None  # e.g. ["lbr_iiwa_14_r820"]
start_rviz = False
### For test debugging ###

package_name = "kuka_ros2_control_support"
launch_file_name = "test_bringup.launch.py"
robot_names_7dof = ["lbr_iiwa_14_r820"]
description_package_arg_name = "description_package"
description_macro_file_arg_name = "description_macro_file"
controllers_file_arg_name = "controllers_file"
extra_launch_args = {"use_mock_hardware": "true", "start_rviz": "true" if start_rviz else "false"}
initial_positions_file_arg_name = "initial_positions_file"
robot_names_init_pos_all_zeros = ["kr210l150", "lbr_iiwa_14_r820"]

possible_combinations = get_possible_combinations(
    package_name=package_name,
    launch_file_name=launch_file_name,
    robot_names_7dof=robot_names_7dof,
    description_package_arg_name=description_package_arg_name,
    description_macro_file_arg_name=description_macro_file_arg_name,
    controllers_file_arg_name=controllers_file_arg_name,
    initial_positions_file_arg_name=initial_positions_file_arg_name,
    robot_names_init_pos_all_zeros=robot_names_init_pos_all_zeros,
)

launch_testing_params = [
    (
        f"{description_package_arg_name}={combination.description_package} "
        f"{description_macro_file_arg_name}={combination.description_macro_file} "
        f"{controllers_file_arg_name}={combination.controllers_file} "
        f"{initial_positions_file_arg_name}={combination.init_pos_file}"
    )
    for combination in possible_combinations
    if not test_only_these_robot_names or combination.robot_name in test_only_these_robot_names
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
    global initial_positions_file_arg_name
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
            f"{initial_positions_file_arg_name}": launch_args[initial_positions_file_arg_name],
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
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.traj_state_topic = "/position_trajectory_controller/state"
        cls.joint_trajectory_topic = "/position_trajectory_controller/joint_trajectory"
        cls.trajectory_publisher = cls.node.create_publisher(
            JointTrajectory, cls.joint_trajectory_topic, 10
        )
        cls.joint_state_topic = "/joint_states"
        cls.node.get_logger().info("Setup complete")

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.get_logger().info("Tearing down")
        cls.node.destroy_node()
        rclpy.shutdown()

    #
    # Test functions
    #

    def test_node_started(self):
        # Verify that the node started successfully
        node_names = self.node.get_node_names()
        self.assertIn(self.node_name, node_names)
        self.node.get_logger().info("Node started successfully")

    def test_joint_state_msg_retrieval(self, time_to_wait=3.0):
        ret, _ = wait_for_message(
            JointState,
            self.node,
            self.joint_state_topic,
            time_to_wait=time_to_wait,
        )
        self.assertTrue(
            ret, f"failed to receive a joint state message from topic '{self.joint_state_topic}'"
        )

    def test_trajectory_state_retrieval(self):
        ret, _ = self.get_trajectory_state()
        self.assertTrue(
            ret,
            f"failed to receive a trajectory state message from topic '{self.traj_state_topic}'",
        )

    def test_tcp_tf(self):
        self.node.get_logger().info("Checking TF for TCP availability and consistency")
        ret, tcp_tf = self.get_tcp_transform()
        self.assertTrue(
            ret,
            f"failed to lookup the tcp transform, is robot_state_publisher running ?")
        self.assertTrue(tcp_tf.transform.translation != Vector3(),
                        f"tcp transform translation should not be null")
        self.node.get_logger().info("Checking TF for TCP availability and consistency done")

    def test_trajectory_following(
        self, position_change=0.2, time_from_start=1, almost_equal_places=2
    ):
        def get_joint_states_from_trajectory_state():
            _, msg = self.get_trajectory_state()
            return dict(zip(msg.joint_names, msg.actual.positions))

        self.node.get_logger().info("Waiting for initial joint states")
        joint_states = get_joint_states_from_trajectory_state()

        # Create a trajectory based on the initial joint states
        self.node.get_logger().info("Creating trajectory")
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_states.keys())
        point = JointTrajectoryPoint()
        point.positions = [pos + position_change for pos in joint_states.values()]
        point.time_from_start = Duration(sec=time_from_start)
        trajectory.points.append(point)

        self.node.get_logger().info(f"Publishing trajectory")
        self.trajectory_publisher.publish(trajectory)
        time.sleep(2 * time_from_start)

        self.node.get_logger().info("Waiting for joint states after executed trajectory")
        joint_states = get_joint_states_from_trajectory_state()

        # Check if the actual joint positions match the target ones
        self.node.get_logger().info("Checking joint positions")
        for joint_name, target_position in zip(trajectory.joint_names, point.positions):
            self.assertAlmostEqual(
                joint_states[joint_name], target_position, places=almost_equal_places
            )
        self.node.get_logger().info("Checking joint positions is done")

    #
    # Utility functions
    #

    def get_trajectory_state(self, time_to_wait=3.0):
        return wait_for_message(
            JointTrajectoryControllerState,
            self.node,
            self.traj_state_topic,
            time_to_wait=time_to_wait,
        )

    def get_tcp_transform(self, time_to_wait=5.0, base_link="base_link", tool_link="tool0"):
        tf_buffer = Buffer()
        # spinner thread must be true to get the transforms
        tf_listener = TransformListener(
            buffer=tf_buffer, node=self.node, spin_thread=True)
        t = None
        try:
            ret = tf_buffer.can_transform(
                tool_link,
                base_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=time_to_wait))
            if ret:
                t = tf_buffer.lookup_transform(
                    tool_link,
                    base_link,
                    rclpy.time.Time())
        except TransformException as ex:
            self.node.get_logger().info(
                f'Could not transform {base_link} to {tool_link}: {ex}')

        # required to avoid rclpy.executors.ExternalShutdownException
        tf_listener.__del__()
        if t is not None:
            return (True, t)
        else:
            return (False, None)
