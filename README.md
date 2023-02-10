# Kuka experimental

[![Build Status](http://build.ros.org/job/Kdev__kuka_experimental__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/job/Kdev__kuka_experimental__ubuntu_xenial_amd64)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

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

## Dependencies:
The dependencies are defined inside the `kuka_experimental.repos` file.

## Installation:
1. Update system:
    ```
    sudo apt update
    sudo apt upgrade
    ```
2. Install ROS 2 Rolling:
  * a) Either setup a Docker container with Rolling installed using [RosTeamWS](https://rtw.stoglrobotics.de/master/use-cases/operating_system/create_setup_workspace.html#docker-workspace).
 If you use this method, don't forget to switch to your docker container **before continuing** with the `rtw_switch_to_docker` command.
    
  * b) Or [install ROS 2 Rolling](https://docs.ros.org/en/rolling/Installation.html) directly on your computer.
3. Make sure `colcon` and `vcs` are installed:
    ```
    sudo apt install python3-colcon-common-extensions python3-vcstool
    ```
4. Setup new workspace (_If you used [RosTeamWS](https://rtw.stoglrobotics.de/master/use-cases/operating_system/create_setup_workspace.html) and docker you can skip this step._):
    ```
    mkdir -p ~/workspace/rolling_ws/src  # or go to an existing one
    ```
5. Clone this repo:
    ```
    cd ~/workspace/rolling_ws/src
    git clone -b merge_kuka_experimental_and_kuka_driver git@github.com:StoglRobotics-forks/kuka_experimental.git 
    ```
6. Make sure your base workspace is sourced and update dependencies:
   ```
   source /opt/ros/rolling/setup.bash # source ws
   rosdep update                      # update dependencies
   ```
7. Get the relevant packages and install all additional dependencies:
   ```
   cd ~/workspace/rolling_ws/         # or your workspace base direcotry
   vcs import -w 1 src --skip-existing --input src/kuka_experimental/kuka_experimental.repos 
   rosdep install --ignore-src --from-paths src -y -r
   ```
8. Finally compile everything:
   ```
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```