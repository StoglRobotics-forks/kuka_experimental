# KUKA EKI IO Interface
This package interfaces KUKA IOs over EKI.

The number of simultaneously supported operations is currently set to 2. If you need more, you have to alter
the KRL code and upload it to the controller.

A command takes a pin, a mode and a state for each operation.
The pin is the number of the IO pin on the controller, the mode is the type of the IO (0: do nothing, 1: read input, 
2: write output, 3: read output) and the state is the value of the IO (true: high, false: low).
State is only relevant for writing output pins.

## Example usage as a ROS node

io_example.h

```cpp
#ifndef IO_EXAMPLE
#define IO_EXAMPLE

#include "rclcpp/rclcpp.hpp"
#include <kuka_eki_io_interface/kuka_eki_io_interface.h>
#include <std_srvs/srv/trigger.hpp>

namespace io_example
{
    class IOExample : public rclcpp::Node
    {
        public:
            IOExample();
            ~IOExample() {};

        private:
            std::shared_ptr<kuka_eki_io_interface::KukaEkiIOInterface> _kuka_eki_io_interface;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _example_srv;
            void example_io(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
            void eki_command(int pin1, int pin2, int mode1, int mode2, bool state1, bool state2);
    };
}
#endif
```

io_example.cpp

```cpp
#include <io_example/io_example.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace io_example
{
    Gripper::IOExample() : Node("io_example_node")
    {
        // get parameters
        this->declare_parameter("robot_ip");
        this->declare_parameter("eki_io_port");
        this->declare_parameter("n_io");

        std::string robot_ip;
        int robot_ip;
        this->get_parameter("robot_ip", robot_ip);
        int eki_io_port;
        std::string eki_io_port_str;
        this->get_parameter("eki_io_port", eki_io_port);
        eki_io_port_str = std::to_string(eki_io_port);
        int n_io;
        this->get_parameter("n_io",n_io);

        // initialize kuka_eki_io_interface
        _kuka_eki_io_interface.reset(new kuka_eki_io_interface::KukaEkiIOInterface(robot_ip.c_str(), eki_io_port_str.c_str(), n_io));

        // initialize example service
        _example_srv = this->create_service<std_srvs::srv::Trigger>("example_srv", std::bind(&IOExample::example_io, this, _1, _2));
        RCLCPP_INFO(rclcpp::get_logger("io_example_node"), "Ready to receive commands.");

    }

    void Gripper::eki_command(int pin1, int pin2, int mode1, int mode2, bool state1, bool state2)
    {
        rclcpp::Rate loop_rate(50);
        // set up flags for checking if command was received and executed
        bool f_pin_set = false;
        bool s_pin_set = false;
        bool f_command_received = false;
        bool s_command_received = false;
        bool f_mode_set = false;
        bool s_mode_set = false;
        // set up vectors for reading state
        std::vector<bool> io_states;
        std::vector<int> io_pins;
        std::vector<int> io_types;
        int buff_len;
        // set up vectors for writing command
        const std::vector<int> set_io_pins_cmd{pin1, pin2};
        const std::vector<int> set_io_modes_cmd{mode1, mode2};
        const std::vector<bool> set_target_ios_cmd{ state1, state2};

        // wait until command was received and executed
        bool command_received = false;
        while (!command_received)
        {
            // write command
            _kuka_eki_io_interface->eki_write_command(set_io_pins_cmd, set_io_modes_cmd, set_target_ios_cmd);
            loop_rate.sleep();
            // read state
            _kuka_eki_io_interface->eki_read_state(io_states, io_pins, io_types, buff_len);
            // check if command was received and executed
            f_pin_set = io_pins[0] == set_io_pins_cmd[0];
            s_pin_set = io_pins[1] == set_io_pins_cmd[1];
            f_command_received = io_states[0] == set_target_ios_cmd[0];
            s_command_received = io_states[1] == set_target_ios_cmd[1];
            f_mode_set = io_types[0] == set_io_modes_cmd[0];
            s_mode_set = io_types[1] == set_io_modes_cmd[1];
            command_received = f_pin_set && s_pin_set && f_command_received && s_command_received && f_mode_set && s_mode_set;
        }
    }

    void Gripper::example_io(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
    {
        // set out pin 502 to false, pin 504 to true
        eki_command(502, 504, 2, 2, false, true);
        // read in pin 501 and 503, the state do not matter when reading
        eki_command(501, 503, 1, 1, true, true);
        // set out pin 501 to false, pin 503 to true
        eki_command(501, 503, 2, 2, true, false);
        response->success = true;
    }
}
```