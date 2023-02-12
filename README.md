
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
**Rolling** | [`rolling`](https://github.com/$NAMESPACE$/kuka_experimental/tree/rolling) | [![Rolling Binary Build](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/rolling-binary-build-main.yml/badge.svg?branch=rolling)](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/rolling-binary-build-main.yml?branch=rolling) <br /> [![Rolling Semi-Binary Build](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/rolling-semi-binary-build-main.yml/badge.svg?branch=rolling)](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/rolling-semi-binary-build-main.yml?branch=rolling) | [![Doxygen Doc Deployment](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://$NAMESPACE$.github.io/kuka_experimental_Documentation/rolling/html/index.html) | [kuka_experimental](https://index.ros.org/p/kuka_experimental/#rolling)


ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Foxy** | [`foxy`](https://github.com/$NAMESPACE$/kuka_experimental/tree/foxy) | [![Foxy Binary Build](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/foxy-binary-build-main.yml/badge.svg?branch=foxy)](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/foxy-binary-build-main.yml?branch=foxy) <br /> [![Foxy Semi-Binary Build](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/foxy-semi-binary-build-main.yml/badge.svg?branch=foxy)](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/foxy-semi-binary-build-main.yml?branch=foxy) | [![Doxygen Doc Deployment](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/$NAMESPACE$/kuka_experimental/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://$NAMESPACE$.github.io/kuka_experimental_Documentation/foxy/html/index.html) | [kuka_experimental](https://index.ros.org/p/kuka_experimental/#foxy)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
