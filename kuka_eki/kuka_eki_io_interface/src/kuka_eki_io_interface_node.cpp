#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "kuka_eki_io_interface/kuka_eki_io_interface.h"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("eki_io_node");

    rclcpp::Rate loop_rate(2);
    node->declare_parameter("robot_ip");
    node->declare_parameter("eki_port");
    node->declare_parameter("n_io");

    std::string robot_ip;
    node->get_parameter("robot_ip", robot_ip);
    int eki_port;
    std::string eki_port_;
    node->get_parameter("eki_port", eki_port);
    eki_port_ = std::to_string(eki_port);
    int n_io;
    node->get_parameter("n_io",n_io);
    kuka_eki_io_interface::KukaEkiIOInterface io_interface(robot_ip.c_str(), eki_port_.c_str(), n_io);

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

    RCLCPP_INFO(node->get_logger(), "Shutting down.");
    rclcpp::shutdown();
    return 0;
}