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

#include "kuka_rsi_hw_interface/robot_control_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace kuka_rsi_hw_interface
{

CallbackReturn RobotControlInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger(info_.name), "expecting exactly 1 command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger(info_.name), "expecting only POSITION command interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger(info_.name), "expecting exactly 1 state interface");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger(info_.name), "expecting only POSITION state interface");
      return CallbackReturn::ERROR;
    }
  }

  // RSI
  in_buffer_.resize(1024);  // udp_server.h --> #define BUFSIZE 1024
  out_buffer_.resize(1024);
  remote_host_.resize(1024);
  remote_port_.resize(1024);

  rsi_initial_joint_positions_.resize(6, 0.0);
  rsi_joint_position_corrections_.resize(6, 0.0), ipoc_ = 0;

  local_host_ = info_.hardware_parameters["listen_address"];
  local_port_ = stoi(info_.hardware_parameters["listen_port"]);

  RCLCPP_DEBUG(
    rclcpp::get_logger(info_.name), "robot location: %s:%d", local_host_.c_str(), local_port_);

  // done
  //  status_ = hardware_interface::status::CONFIGURED;
  //  return return_type::OK;
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotControlInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Connecting to robot . . .");

  int bytes = server_->recv(in_buffer_);

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100)
  {
    bytes = server_->recv(in_buffer_);
  }
  if (bytes < 100)
  {
    RCLCPP_FATAL(rclcpp::get_logger(info_.name), "Connection has failed: Not enough data received!");
    return CallbackReturn::ERROR;
  }

  rsi_state_ = kuka_rsi_hw_interface::RSIState(in_buffer_);

  for (size_t i = 0; i < hw_states_.size(); ++i)
  {
    hw_states_[i] = rsi_state_.positions[i] * 3.14159 / 180.0;
    hw_commands_[i] = hw_states_[i];
    rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
  }
  ipoc_ = rsi_state_.ipoc;

  out_buffer_ = kuka_rsi_hw_interface::RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
  server_->send(out_buffer_);
  server_->set_timeout(1000);  // Set receive timeout to 1 second

  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Successfully connected with the robot!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotControlInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotControlInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn RobotControlInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < hw_states_.size(); ++i)
  {
    hw_commands_[i] = hw_states_[i];
  }

  return CallbackReturn::SUCCESS;
}

// return_type RobotControlInterface::stop()
CallbackReturn RobotControlInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(info_.name), "on_deactivate()");

  RCLCPP_DEBUG(rclcpp::get_logger(info_.name), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

// WARN: NOT REAL TIME SAFE due to strings/possible allocations
return_type RobotControlInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  in_buffer_.resize(1024);  // FIXME:
  if (server_->recv(in_buffer_) == 0)
  {  // FIXME: server_->recv is probably doing some allocation
    return return_type::ERROR;
  }

  rsi_state_ = kuka_rsi_hw_interface::RSIState(in_buffer_);

  for (size_t i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = rsi_state_.positions[i] * 3.14159 / 180.0;
    RCLCPP_DEBUG(rclcpp::get_logger(info_.name), "Got state %.5f for joint %ld!", hw_states_[i], i);
  }
  ipoc_ = rsi_state_.ipoc;

  return return_type::OK;
}

return_type RobotControlInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  out_buffer_.resize(1024);  // FIXME

  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger(info_.name), "Got command %.5f for joint %ld!", hw_commands_[i], i);
    rsi_joint_position_corrections_[i] =
      (hw_commands_[i] * 180.0 / 3.14159) - rsi_initial_joint_positions_[i];
  }
  out_buffer_ = kuka_rsi_hw_interface::RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
  server_->send(out_buffer_);

  return return_type::OK;
}

}  // namespace kuka_rsi_hw_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kuka_rsi_hw_interface::RobotControlInterface, hardware_interface::SystemInterface)
