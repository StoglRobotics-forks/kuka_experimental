// Copyright (c) 2021 Gergely Sóti
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "kuka_eki_io_interface/kuka_eki_io_interface.h"


int main(int argc,  char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("eki_io_node");

    rclcpp::Rate loop_rate(2);
    node->declare_parameter("robot_ip", "0.0.0.0");
    node->declare_parameter("eki_io_port", 54601);
    node->declare_parameter("n_io", 2);

    std::string robot_ip = node->get_parameter("robot_ip").as_string();
    int eki_io_port = node->get_parameter("eki_io_port").as_int();
    std::string eki_io_port_ = std::to_string(eki_io_port);
    int n_io = node->get_parameter("n_io").as_int();
    kuka_eki_io_interface::KukaEkiIOInterface io_interface(robot_ip.c_str(), eki_io_port_.c_str(), n_io);
//
    const std::vector<int> io_pins_cmd{7, 8};
    const std::vector<int> io_modes_cmd{2, 2};
    const std::vector<bool> target_ios_cmd{ false, true};
//    io_interface.eki_write_command(io_pins_cmd, io_modes_cmd, target_ios_cmd);

    loop_rate.sleep();
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        io_interface.eki_write_command(io_pins_cmd, io_modes_cmd, target_ios_cmd);

        rclcpp::spin_some(node);
        std::vector<bool> io_states;
        std::vector<int> io_pins;
        std::vector<int> io_types;
        int buff_len;
        io_interface.eki_read_state(io_states, io_pins, io_types, buff_len);
//        for(int i = 0; i < n_io; i++)
//        {
//            RCLCPP_INFO(node->get_logger(), "IO state: %d,  %d,  %d", io_pins[i], io_types[1], io_states[i]);
//        }
        loop_rate.sleep();
    }
    for(int i = 0; i<10; i++) {
        io_interface.eki_write_command(io_pins_cmd, io_modes_cmd, target_ios_cmd);
    }

    RCLCPP_INFO(rclcpp::get_logger("eki_io_node"), "Shutting down.");
    rclcpp::shutdown();
    return 0;
}