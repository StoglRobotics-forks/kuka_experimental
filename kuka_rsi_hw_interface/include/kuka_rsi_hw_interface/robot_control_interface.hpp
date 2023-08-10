/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef KUKA_RSI_HW_INTERFACE__ROBOT_CONTROL_INTERFACE_HPP_
#define KUKA_RSI_HW_INTERFACE__ROBOT_CONTROL_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "kuka_rsi_hw_interface/visibility_control.hpp"

#include "kuka_rsi_hw_interface/rsi_command.hpp"
#include "kuka_rsi_hw_interface/rsi_state.hpp"
#include "kuka_rsi_hw_interface/udp_server.hpp"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kuka_rsi_hw_interface
{
class RobotControlInterface : public hardware_interface::SystemInterface
{
public:
  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  // return_type configure(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  // return_type start() override;

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  // return_type stop() override;

  ROS2_CONTROL_DRIVER_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Dummy parameters
  double hw_start_sec_, hw_stop_sec_, hw_slowdown_;
  // Store the command for the simulated robot
  std::vector<double> hw_commands_, hw_states_;

  // RSI
  kuka_rsi_hw_interface::RSIState rsi_state_;
  kuka_rsi_hw_interface::RSICommand rsi_command_;
  std::vector<double> rsi_initial_joint_positions_;
  std::vector<double> rsi_joint_position_corrections_;
  unsigned long long ipoc_;

  std::unique_ptr<UDPServer> server_;
  std::string local_host_;
  int local_port_;
  std::string remote_host_;
  std::string remote_port_;
  std::string in_buffer_;
  std::string out_buffer_;

  double loop_hz_;
  // double control_period_;
  std::chrono::steady_clock::time_point time_now_, time_last_;
  std::chrono::duration<double> control_period_, elapsed_time_;
  // rclcpp::Duration control_period_;
  // rclcpp::Duration elapsed_time_;
};

}  // namespace kuka_rsi_hw_interface

#endif  // KUKA_RSI_HW_INTERFACE__ROBOT_CONTROL_INTERFACE_HPP_
