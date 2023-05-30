// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of 3M nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef KUKA_RSI_HW_INTERFACE__VISIBILITY_CONTROL_HPP_
#define KUKA_RSI_HW_INTERFACE__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ROS2_CONTROL_DRIVER_EXPORT __attribute__((dllexport))
#define ROS2_CONTROL_DRIVER_IMPORT __attribute__((dllimport))
#else
#define ROS2_CONTROL_DRIVER_EXPORT __declspec(dllexport)
#define ROS2_CONTROL_DRIVER_IMPORT __declspec(dllimport)
#endif
#ifdef ROS2_CONTROL_DRIVER_BUILDING_DLL
#define ROS2_CONTROL_DRIVER_PUBLIC ROS2_CONTROL_DRIVER_EXPORT
#else
#define ROS2_CONTROL_DRIVER_PUBLIC ROS2_CONTROL_DRIVER_IMPORT
#endif
#define ROS2_CONTROL_DRIVER_PUBLIC_TYPE ROS2_CONTROL_DRIVER_PUBLIC
#define ROS2_CONTROL_DRIVER_LOCAL
#else
#define ROS2_CONTROL_DRIVER_EXPORT __attribute__((visibility("default")))
#define ROS2_CONTROL_DRIVER_IMPORT
#if __GNUC__ >= 4
#define ROS2_CONTROL_DRIVER_PUBLIC __attribute__((visibility("default")))
#define ROS2_CONTROL_DRIVER_LOCAL __attribute__((visibility("hidden")))
#else
#define ROS2_CONTROL_DRIVER_PUBLIC
#define ROS2_CONTROL_DRIVER_LOCAL
#endif
#define ROS2_CONTROL_DRIVER_PUBLIC_TYPE
#endif

#endif  // KUKA_RSI_HW_INTERFACE__VISIBILITY_CONTROL_HPP_
