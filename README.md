
# Kuka experimental

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

Experimental packages for Kuka manipulators within [ROS-Industrial][].
See the [ROS wiki][] page for more information.

This repository contains packages that will be migrated to the [kuka][]
repository after they have received sufficient testing. The contents of
these packages are subject to change, without prior notice. Any available
APIs are to be considered unstable and are not guaranteed to be complete
and / or functional.

[ROS-Industrial]: http://wiki.ros.org/Industrial
[ROS wiki]: http://wiki.ros.org/kuka_experimental
[kuka]: https://github.com/ros-industrial/kuka

## Contents

KUKA is one the major robot manufacturers in the world. Therefore, KUKA robots can be found in many industrial applications and research labs.

This branch contains ROS 2 version of the driver supporting new features enabled by ROS 2, such as decreased latency, improved security, and more flexibility regarding middleware configuration.
The repository contains descriptions for many KUKA manipulators, ros2_control support for RSI and EKI Protocols to control the robot, as well as generic MoveIt2 setup to get you started.

[//]: <> (TODO: I you want to learn more check the videos and presentation about the repository:)


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

## Packages in the Repository:

- `kuka_experimental` - Meta package that provides a single point of installation for the released packages.
- `kuka_eki_hw_interface` - Hardware interface implementation for ros2_control based on EKI communication protocol.
- `kuka_rsi_hw_interface` - Hardware interface implementation for ros2_control based on RSI communication protocol.
- `kuka_resources` - Shared resources across support packages, e.g., rviz configuration, launch files, and URDF properties (colors, materials, math constants), etc.
- `kuka_ros2_control_support` - Support files for using KUKA with ros2_control.
- `kuka_rsi_simulator` - Simple simulator of RSI communication that is useful for driver testing without connected hardware.

- **support packages** - descriptions, meshes and test-launch file for different versions of KUKA manipulators.

  - `kuka_kr3_support`
  - `kuka_kr5_support`
  - `kuka_kr6_support`
  - `kuka_kr10_support`
  - `kuka_kr16_support`
  - `kuka_kr120_support`
  - `kuka_kr150_support`
  - `kuka_kr210_support`
  - `kuka_lbr_iiwa_support` - *NOT tested extensively! Note that there are packages with possibly better support!*


## Getting Started

For getting started to use this driver you have to follow the next steps:

1. **Install the driver** (see below). Currently only install from source is supported.

2. **Setup the robot**. Once driver is installed, decide which interface, RSI or EKI, you want to use and [check instructions for RSI interface](./kuka_rsi_hw_interface/krl/README.md) or [check instructions for EKI interface](./kuka_eki_hw_interface/krl/README.md).

3. **Start the driver** (see below).


### Build from Source

Before building from source please make sure that you actually need to do that. Building from source
might require some special treatment, especially when it comes to dependency management.
Dependencies might change from time to time. Upstream packages might change
their features / API which require changes in this repo. Therefore, this repo's source builds might
require upstream repositories to be present in a certain version as otherwise builds might fail.
Starting from scratch following exactly the steps below should always work, but simply pulling and
building might fail occasionally.

1. [Install ROS2](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html).
   For using this driver with ROS2 `foxy`. Checkout [foxy
   branch](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy), for using it
   with ROS2 ``humble``, use the [humble branch](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble),
   and for using it with ROS2 ``rolling``, use the [rolling
   branch](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/rolling).

   Once installed, please make sure to actually [source ROS2](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files) before proceeding.

3. Make sure that `colcon`, its extensions and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```

4. Create a new ROS2 workspace:
   ```
   export COLCON_WS=~/workspace/ros2_kuka_driver
   mkdir -p $COLCON_WS/src
   ```

5. Clone relevant packages, install dependencies, compile, and source the workspace by using:
   ```
   cd $COLCON_WS
   git clone https://github.com/StoglRobotics-forks/kuka_experimental.git src/kuka_experimental
   vcs import src --skip-existing --input src/kuka_experimental/kuka_experimental-not-released.${ROS_DISTRO}.repos
   rosdep update
   rosdep install --ignore-src --from-paths src -y
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

6. When consecutive pulls leads to build errors, please make sure to update the upstream packages before
   filing an issue:
   ```
   cd $COLCON_WS
   vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
   rosdep update
   rosdep install --ignore-src --from-paths src -y
   ```


### Starting the driver

1. Start *Mock Hardware* first to see how the system is setup (example for `KR5 arc`):
   ```
   ros2 launch kuka_ros2_control_support test_bringup.launch.py description_package:=kuka_kr5_support description_macro_file:=kr5_arc_macro.xacro use_mock_hardware:=true
   ```
   Now check started nodes, available topics and services.

2. Start test node for joint trajectory controller:
   ```
   ros2 launch kuka_ros2_control_support test_joint_trajectory_controller.launch.py
   ```

3. To start other robot type use the following arguments:
    ```
    "description_package",
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

    "description_macro_file",
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

    "controllers_file",
        choices=[
            "kuka_6dof_controllers.yaml",
            "kuka_7dof_controllers.yaml",
            # Note: for the robot kuka_lbr_iiwa_14_r820, kuka_7dof_controllers.yaml should be used
            # and the rest use kuka_6dof_controllers.yaml
        ]

    ```
    **NOTE: Please choose only the related combination of the various parameters.**

4. Start the real hardware using the following arguments on `test_bringup.launch.py`:
   ```
   use_rsi_communication:=true rsi_listen_ip:=<IP_OF_PC_IN_RSI_Network> rsi_listen_port:=49152
   ```
   You can omit `rsi_listen_port` if you didn't change it in the robots configuration.
   For example:
   ```
   ```
