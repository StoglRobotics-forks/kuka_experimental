
# Kuka experimental

[![Build Status: Ubuntu Bionic (Actions)](https://github.com/ros-industrial/kuka_experimental/workflows/CI%20-%20Ubuntu%20Bionic/badge.svg?branch=melodic-devel)](https://github.com/ros-industrial/kuka_experimental/actions?query=workflow%3A%22CI+-+Ubuntu+Bionic%22)
[![Build Status: Ubuntu Focal (Actions)](https://github.com/ros-industrial/kuka_experimental/workflows/CI%20-%20Ubuntu%20Focal/badge.svg?branch=melodic-devel)](https://github.com/ros-industrial/kuka_experimental/actions?query=workflow%3A%22CI+-+Ubuntu+Focal%22)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

Experimental packages for Kuka manipulators within [ROS-Industrial][].
See the [ROS wiki][] page for more information.

## Contents

This repository contains packages that will be migrated to the [kuka][]
repository after they have received sufficient testing. The contents of
these packages are subject to change, without prior notice. Any available
APIs are to be considered unstable and are not guaranteed to be complete
and / or functional.

[ROS-Industrial]: http://wiki.ros.org/Industrial
[ROS wiki]: http://wiki.ros.org/kuka_experimental
[kuka]: https://github.com/ros-industrial/kuka

## Build status

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`rolling`](https://github.com/StoglRobotics-forks/kuka_experimental/tree/rolling) | [![Rolling Binary Build](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/rolling-binary-build-main.yml/badge.svg?branch=rolling)](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/rolling-binary-build-main.yml?branch=rolling) <br /> [![Rolling Semi-Binary Build](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/rolling-semi-binary-build-main.yml/badge.svg?branch=rolling)](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/rolling-semi-binary-build-main.yml?branch=rolling) | [![Doxygen Doc Deployment](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://StoglRobotics-forks.github.io/kuka_experimental_Documentation/rolling/html/index.html) | [kuka_experimental](https://index.ros.org/p/kuka_experimental/#rolling)
**Foxy** | [`foxy`](https://github.com/StoglRobotics-forks/kuka_experimental/tree/foxy) | [![Foxy Binary Build](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/foxy-binary-build-main.yml/badge.svg?branch=foxy)](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/foxy-binary-build-main.yml?branch=foxy) <br /> [![Foxy Semi-Binary Build](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/foxy-semi-binary-build-main.yml/badge.svg?branch=foxy)](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/foxy-semi-binary-build-main.yml?branch=foxy) | [![Doxygen Doc Deployment](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/StoglRobotics-forks/kuka_experimental/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://StoglRobotics-forks.github.io/kuka_experimental_Documentation/foxy/html/index.html) | [kuka_experimental](https://index.ros.org/p/kuka_experimental/#foxy)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.


## How to use this repository

1. Create a colcon workspace:

    ```
    # update and upgrade
    sudo apt update
    sudo apt upgrade -y

    # Setup the workspace
    mkdir -p ~/ws/src
    cd ~/ws/src

    # Download needed software
    git clone https://github.com/StoglRobotics-forks/kuka_experimental.git -b kuka_ros2_control_sim_support-package-added


    # importing dependent repos
    cd ~/ws
    vcs import src < src/kuka_experimental/kuka_experimental.rolling.repos
    ```

1. Install dependencies:

    ```
    cd ~/ws
    rosdep install -r --from-paths src -i -y --rosdistro rolling
    ```

1. Build the workspace:

    ```
    # Source ROS distro's setup.bash
    source /opt/ros/<distro>/setup.bash

    # Build and install into workspace
    cd ~/ws
    colcon build
    ```

1. Launch possibilities  :

    ```
    # launches only robot model with Joint state publisher GUI
    ros2 launch kuka_kr3_support test_kr3r540.launch.py

    # possible arguments for the following launch file are as follows: 

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
            ]
    
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
            ]
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
            ]
    "controllers_file",
            choices=[
                "kuka_6dof_controllers.yaml",
                "kuka_7dof_controllers.yaml",
                # Note: for the robot kuka_lbr_iiwa_14_r820, kuka_7dof_controllers.yaml should be used
                # and the rest use kuka_6dof_controllers.yaml
            ]

    NOTE: Please choose only the related combination of the various parameters.

    # launches robot model with ros2_control support for "kuka_kr16_2" robot
    ros2 launch kuka_ros2_control_support test_ros2_control_kuka.launch.py robot_description_package:=kuka_kr16_support robot_description_macro_file:=kr16_2_macro.xacro robot_name:=kuka_kr16_2 controllers_file:=kuka_6dof_controllers.yaml

    ```
