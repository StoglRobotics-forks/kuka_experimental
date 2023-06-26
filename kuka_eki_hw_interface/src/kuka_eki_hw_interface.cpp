// Copyright 2020 ros2_control Development Team
// Modifications copyright (c) 2021 Gergely Sóti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kuka_eki_hw_interface/kuka_eki_hw_interface.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kuka_eki_hw_interface
{

    KukaEkiHardwareInterface::~KukaEkiHardwareInterface()
    {
      // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
      // We therefore need to make sure to actually deactivate the communication
      on_deactivate(rclcpp_lifecycle::State());
    }

    hardware_interface::CallbackReturn KukaEkiHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        info_ = info;

        hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            // RRBotSystemPositionOnly has exactly one state and command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("KukaEkiHardwareInterface"),
                    "Joint '%s' has %ld command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("KukaEkiHardwareInterface"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("KukaEkiHardwareInterface"),
                    "Joint '%s' has %ld state interface. 1 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("KukaEkiHardwareInterface"),
                    "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

//        status_ = hardware_interface::status::CONFIGURED;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> KukaEkiHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> KukaEkiHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    void KukaEkiHardwareInterface::eki_check_read_state_deadline()
    {
        // Check if deadline has already passed
        if (deadline_->expires_at() <= boost::asio::deadline_timer::traits_type::now())
        {
            eki_server_socket_->cancel();
            deadline_->expires_at(boost::posix_time::pos_infin);
        }

        // Sleep until deadline exceeded
        deadline_->async_wait(boost::bind(&KukaEkiHardwareInterface::eki_check_read_state_deadline, this));
    }

    void KukaEkiHardwareInterface::eki_handle_receive(const boost::system::error_code &ec, size_t length,
                                                  boost::system::error_code* out_ec, size_t* out_length)
    {
        *out_ec = ec;
        *out_length = length;
    }

    bool KukaEkiHardwareInterface::eki_read_state(std::vector<double> &joint_position,
                                                  std::vector<double> &joint_velocity,
                                                  std::vector<double> &joint_effort,
                                                  int &cmd_buff_len)
    {
        static boost::array<char, 2048> in_buffer;
        // Read socket buffer (with timeout)
        // Based off of Boost documentation example: doc/html/boost_asio/example/timeouts/blocking_udp_client.cpp
        deadline_->expires_from_now(boost::posix_time::seconds(eki_read_state_timeout_));  // set deadline
        boost::system::error_code ec = boost::asio::error::would_block;
        size_t len = 0;

        eki_server_socket_->async_receive(boost::asio::buffer(in_buffer),
                                       boost::bind(&KukaEkiHardwareInterface::eki_handle_receive, _1, _2, &ec, &len));
        do
            ios_.run_one();
        while (ec == boost::asio::error::would_block);

        if (ec)
        {
            RCLCPP_WARN(rclcpp::get_logger("KukaEkiHardwareInterface"), " communication error code: %s", ec.message().c_str());
            return false;
        }

        // Update joint positions from XML packet (if received)
        if (len == 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("KukaEkiHardwareInterface"), " packet of len 0 received");
            return false;
        }

        // Parse XML
        TiXmlDocument xml_in;
        in_buffer[len] = '\0';  // null-terminate data buffer for parsing (expects c-string)
        xml_in.Parse(in_buffer.data());
        TiXmlElement* robot_state = xml_in.FirstChildElement("RobotState");
        if (!robot_state)
            return false;
        TiXmlElement* pos = robot_state->FirstChildElement("Pos");
        TiXmlElement* vel = robot_state->FirstChildElement("Vel");
        TiXmlElement* eff = robot_state->FirstChildElement("Eff");
        TiXmlElement* robot_command = robot_state->FirstChildElement("RobotCommand");
        if (!pos || !vel || !eff || !robot_command)
        {
            RCLCPP_INFO(rclcpp::get_logger("KukaEkiHardwareInterface"), "parsing failed, one of pos, vel eff or RobotCommand missing");
            return false;
        }

        // Extract axis positions
        double joint_pos;  // [deg]
        double joint_vel;  // [%max]
        double joint_eff;  // [Nm]
        char axis_name[] = "A1";
        for (long unsigned int i = 0; i < hw_states_.size(); ++i)
        {
            pos->Attribute(axis_name, &joint_pos);
            joint_position[i] = angles::from_degrees(joint_pos);  // convert deg to rad
            vel->Attribute(axis_name, &joint_vel);
            joint_velocity[i] = joint_vel;
            eff->Attribute(axis_name, &joint_eff);
            joint_effort[i] = joint_eff;
            axis_name[1]++;
        }

        // Extract number of command elements buffered on robot
        robot_command->Attribute("Size", &cmd_buff_len);
        return true;
    }

    bool KukaEkiHardwareInterface::eki_write_command(const std::vector<double> &joint_position_command)
    {
        TiXmlDocument xml_out;
        TiXmlElement* robot_command = new TiXmlElement("RobotCommand");
        TiXmlElement* pos = new TiXmlElement("Pos");
        TiXmlText* empty_text = new TiXmlText("");
        robot_command->LinkEndChild(pos);
        pos->LinkEndChild(empty_text);   // force <Pos></Pos> format (vs <Pos />)
        char axis_name[] = "A1";
        for (long unsigned int i = 0; i < hw_states_.size(); ++i)
        {
            pos->SetAttribute(axis_name, std::to_string(angles::to_degrees(joint_position_command[i])).c_str());
            axis_name[1]++;
        }
        xml_out.LinkEndChild(robot_command);
        TiXmlPrinter xml_printer;
        xml_printer.SetStreamPrinting();  // no linebreaks
        xml_out.Accept(&xml_printer);

        eki_server_socket_->send_to(boost::asio::buffer(xml_printer.CStr(), xml_printer.Size()),
                                              eki_server_endpoint_);

      return true;
    }

    hardware_interface::CallbackReturn KukaEkiHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("KukaEkiHardwareInterface"), "Starting ... please wait...");

        eki_server_address_ = info_.hardware_parameters["robot_ip"];
        eki_server_port_ = info_.hardware_parameters["eki_robot_port"];
        RCLCPP_INFO(rclcpp::get_logger("KukaEkiHardwareInterface"), "using IP: %s", eki_server_address_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("KukaEkiHardwareInterface"), "using port: %s", eki_server_port_.c_str());

        if (eki_server_address_.empty() || eki_server_port_.empty())
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("KukaEkiHardwareInterface"), "robot_ip or eki_robot_port cannot be empty");
            return hardware_interface::CallbackReturn::ERROR;
        }

        deadline_.reset(new boost::asio::deadline_timer(ios_));
        eki_server_socket_.reset(new boost::asio::ip::udp::socket(ios_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)));

        boost::asio::ip::udp::resolver resolver(ios_);
        eki_server_endpoint_ = *resolver.resolve({boost::asio::ip::udp::v4(), eki_server_address_, eki_server_port_});

        boost::array<char, 1> ini_buf = { 0 };
        eki_server_socket_->send_to(boost::asio::buffer(ini_buf), eki_server_endpoint_);  // initiate contact to start server

        // Start persistent actor to check for eki_read_state timeouts
        deadline_->expires_at(boost::posix_time::pos_infin);  // do nothing until a read is invoked (deadline_ = +inf)
        eki_check_read_state_deadline();

        std::vector<double> joint_position;
        std::vector<double> joint_velocity;
        std::vector<double> joint_effort;
        joint_position.resize(hw_states_.size());
        joint_velocity.resize(hw_states_.size());
        joint_effort.resize(hw_states_.size());
        if (!eki_read_state(joint_position, joint_velocity, joint_effort, eki_cmd_buff_len_))
        {
            std::string msg = "Failed to read from robot EKI server within alloted time of "
                              + std::to_string(eki_read_state_timeout_) + " seconds.  Make sure eki_hw_interface is running "
                              "on the robot controller and all configurations are correct.";
            RCLCPP_ERROR(
                rclcpp::get_logger("KukaEkiHardwareInterface"), msg.c_str());
            throw std::runtime_error(msg);
        }
        hw_commands_ = joint_position;
        hw_states_ = joint_position;

        RCLCPP_INFO(rclcpp::get_logger("KukaEkiHardwareInterface"), "System Successfully started!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KukaEkiHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("KukaEkiHardwareInterface"), "Stopping ...please wait...");

        RCLCPP_INFO(rclcpp::get_logger("KukaEkiHardwareInterface"), "System successfully stopped!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type KukaEkiHardwareInterface::read(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
    {
        std::vector<double> joint_position;
        std::vector<double> joint_velocity;
        std::vector<double> joint_effort;
        joint_position.resize(hw_states_.size());
        joint_velocity.resize(hw_states_.size());
        joint_effort.resize(hw_states_.size());
        if (!eki_read_state(joint_position, joint_velocity, joint_effort, eki_cmd_buff_len_))
        {
            std::string msg = "Failed to read from robot EKI server within alloted time of "
                              + std::to_string(eki_read_state_timeout_) + " seconds.  Make sure eki_hw_interface is running "
                              "on the robot controller and all configurations are correct.";
            RCLCPP_ERROR(
                rclcpp::get_logger("KukaEkiHardwareInterface"), msg.c_str());
            throw std::runtime_error(msg);
        }
        hw_states_ = joint_position;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KukaEkiHardwareInterface::write(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
    {
        if (eki_cmd_buff_len_ < eki_max_cmd_buff_len_)
            eki_write_command(hw_commands_);
        return hardware_interface::return_type::OK;
    }

}  // namespace kuka_eki_hw_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    kuka_eki_hw_interface::KukaEkiHardwareInterface, hardware_interface::SystemInterface)