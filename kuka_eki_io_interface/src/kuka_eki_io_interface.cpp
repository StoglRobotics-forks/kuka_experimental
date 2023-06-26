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

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <tinyxml.h>

#include <kuka_eki_io_interface/kuka_eki_io_interface.h>

namespace kuka_eki_io_interface
{
    KukaEkiIOInterface::KukaEkiIOInterface(const std::string& eki_server_address, const std::string& eki_server_port, int n_io) : deadline_(ios_),
    eki_server_socket_(ios_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
    {
        eki_server_address_ = eki_server_address;
        eki_server_port_ = eki_server_port;
        n_io_ = n_io;

        std::cout << eki_server_address_ << std::endl;
        std::cout << eki_server_port_ << std::endl;
        std::cout << n_io_ << std::endl;

        boost::asio::ip::udp::resolver resolver(ios_);
        eki_server_endpoint_ = *resolver.resolve({boost::asio::ip::udp::v4(), eki_server_address_, eki_server_port_});

        boost::array<char, 1> ini_buf = { 0 };
        eki_server_socket_.send_to(boost::asio::buffer(ini_buf), eki_server_endpoint_);

        deadline_.expires_at(boost::posix_time::pos_infin);  // do nothing unit a read is invoked (deadline_ = +inf)
        eki_check_read_state_deadline();

//        std::vector<bool> io_states;
//        std::vector<int> io_pins;
//        std::vector<int> io_types;
//        int buff_len;
//        if (!eki_read_state(io_states, io_pins, io_types, buff_len))
//        {
//            std::string msg = "Failed to read from robot EKI server within alloted time of "
//                              + std::to_string(eki_read_state_timeout_) + " seconds.  Make sure eki_hw_interface is running "
//                              "on the robot controller and all configurations are correct.";
//            std::cout << msg << std::endl;
//            throw std::runtime_error(msg);
//        }
    }
//    KukaEkiIOInterface::~KukaEkiIOInterface() {}

    void KukaEkiIOInterface::eki_check_read_state_deadline()
    {
        // Check if deadline has already passed
        if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
        {
            eki_server_socket_.cancel();
            deadline_.expires_at(boost::posix_time::pos_infin);
        }
        // Sleep until deadline exceeded
        deadline_.async_wait(boost::bind(&KukaEkiIOInterface::eki_check_read_state_deadline, this));
    }

    bool KukaEkiIOInterface::eki_read_state(std::vector<bool> &io_states, std::vector<int> &io_pins, std::vector<int> &io_types, int &cmd_buff_len)
    {
        io_states.resize(n_io_);
        io_pins.resize(n_io_);
        io_types.resize(n_io_);
        static boost::array<char, 2048> in_buffer;
        deadline_.expires_from_now(boost::posix_time::seconds(eki_read_state_timeout_));  // set deadline
        boost::system::error_code ec = boost::asio::error::would_block;
        size_t len = 0;
        eki_server_socket_.async_receive(boost::asio::buffer(in_buffer),
                               boost::bind(&KukaEkiIOInterface::eki_handle_receive, _1, _2, &ec, &len));

        do {
            ios_.run_one();
            }
        while (ec == boost::asio::error::would_block);
        if (ec)
        {
            std::cout << ec << std::endl;
            return false;
        }
        if (len == 0)
        {
            std::cout << "2" << std::endl;
            return false;
        }

        TiXmlDocument xml_in;
        in_buffer[len] = '\0';  // null-terminate data buffer for parsing (expects c-string)
        xml_in.Parse(in_buffer.data());
        TiXmlElement* robot_state = xml_in.FirstChildElement("IOState");
        if (!robot_state)
        {
            std::cout << "3" << std::endl;
            return false;
        }
        xml_in.Print();
        char io_name[] = "IO1";
        for (int i = 0; i < n_io_; ++i)
        {
            TiXmlElement* state = robot_state->FirstChildElement(io_name);
            if (!state)
            {
                std::cout << "4" << std::endl;
                return false;
            }
            int io_state;  // [Nm]
            state->Attribute("State", &io_state);
            int io_pin;
            state->Attribute("Pin", &io_pin);
            int io_type;
            state->Attribute("Type", &io_type);
            io_states[i] = io_state;
            io_pins[i] = io_pin;
            io_types[i] = io_type;
            io_name[2]++;
        }
        TiXmlElement* robot_command = robot_state->FirstChildElement("IOCommand");
        robot_command->Attribute("Size", &cmd_buff_len);
        return true;
    }

    bool KukaEkiIOInterface::eki_write_command(const std::vector<int> &io_pins, const std::vector<int> &io_modes, const std::vector<bool> &target_ios)
    {
        // TODO: assert vectors' lengths
        // TODO: extend vector if length < n_io_
        TiXmlDocument xml_out;
        TiXmlElement* io_command = new TiXmlElement("IOCommand");
        char io_name[] = "IO1";
        for (int i = 0; i < n_io_; i++)
        {
            TiXmlElement* io_element = new TiXmlElement(io_name);
            TiXmlText* empty_text = new TiXmlText("");
            io_command->LinkEndChild(io_element);
            io_element->LinkEndChild(empty_text);

            io_element->SetAttribute("Pin", io_pins[i]);
            io_element->SetAttribute("Mode", io_modes[i]);
            io_element->SetAttribute("Value", (int)target_ios[i]);

            io_name[2]++;
        }
        xml_out.LinkEndChild(io_command);
        xml_out.Print();
        TiXmlPrinter xml_printer;
        xml_printer.SetStreamPrinting();  // no linebreaks
        xml_out.Accept(&xml_printer);

        eki_server_socket_.send_to(boost::asio::buffer(xml_printer.CStr(), xml_printer.Size()),
                                              eki_server_endpoint_);
        return true;
    }

    void KukaEkiIOInterface::eki_handle_receive(const boost::system::error_code &ec, size_t length,
                                                  boost::system::error_code* out_ec, size_t* out_length)
    {
        *out_ec = ec;
        *out_length = length;
    }
}