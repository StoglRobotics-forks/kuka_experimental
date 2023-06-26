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

#ifndef ROS2_CONTROL_KUKA_EKI_HW__KUKA_EKI_POSITION_ONLY_HPP_
#define ROS2_CONTROL_KUKA_EKI_HW__KUKA_EKI_POSITION_ONLY_HPP_

#include <memory>
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include "angles/angles.h"

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

#include "tinyxml.h"
//#include "hardware_interface/system_interface.hpp"
//#include "hardware_interface/handle.hpp"
//#include "hardware_interface/hardware_info.hpp"
//#include "hardware_interface/system_interface.hpp"
//#include "hardware_interface/types/hardware_interface_return_values.hpp"
//#include "hardware_interface/types/lifecycle_state_names.hpp"
//#include "rclcpp/macros.hpp"
//#include "kuka_eki_hw_interface/visibility_control.h"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace kuka_eki_hw_interface
{
    class KukaEkiHardwareInterface : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(KukaEkiHardwareInterface)
            virtual ~KukaEkiHardwareInterface();

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

            hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
            hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

//            ROS2_CONTROL_KUKA_EKI_HW_PUBLIC
//            hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info);
//
//            ROS2_CONTROL_KUKA_EKI_HW_PUBLIC
//            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
//
//            ROS2_CONTROL_KUKA_EKI_HW_PUBLIC
//            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
//
//            ROS2_CONTROL_KUKA_EKI_HW_PUBLIC
//            hardware_interface::return_type start();
//
//            ROS2_CONTROL_KUKA_EKI_HW_PUBLIC
//            hardware_interface::return_type stop();
//
//            ROS2_CONTROL_KUKA_EKI_HW_PUBLIC
//            hardware_interface::return_type read();
//
//            ROS2_CONTROL_KUKA_EKI_HW_PUBLIC
//            hardware_interface::return_type write();

        private:
            // Store the command for the simulated robot
            std::vector<double> hw_commands_;
            std::vector<double> hw_states_;

            std::string eki_server_address_;
            std::string eki_server_port_;

            int eki_cmd_buff_len_;
            int eki_max_cmd_buff_len_ = 5;  // by default, limit command buffer to 5 (size of advance run in KRL)
//
//            // EKI socket read/write
            int eki_read_state_timeout_ = 10;  // [s]; settable by parameter (default = 5)
            boost::asio::io_service ios_;
            std::unique_ptr<boost::asio::deadline_timer> deadline_;
            boost::asio::ip::udp::endpoint eki_server_endpoint_;
            std::unique_ptr<boost::asio::ip::udp::socket> eki_server_socket_;
            void eki_check_read_state_deadline();
            static void eki_handle_receive(const boost::system::error_code &ec, size_t length,
                                         boost::system::error_code* out_ec, size_t* out_length);
            bool eki_read_state(std::vector<double> &joint_position, std::vector<double> &joint_velocity,
                              std::vector<double> &joint_effort, int &cmd_buff_len);
            bool eki_write_command(const std::vector<double> &joint_position);
    };

}  // namespace kuka_eki_hw_interface

#endif  // ROS2_CONTROL_KUKA_EKI_HW__KUKA_EKI_POSITION_ONLY_HPP_