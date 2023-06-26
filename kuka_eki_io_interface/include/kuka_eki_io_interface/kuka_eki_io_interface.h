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

#ifndef KUKA_EKI_IO_INTERFACE
#define KUKA_EKI_IO_INTERFACE

#include <vector>
#include <string>

#include <boost/asio.hpp>

namespace kuka_eki_io_interface
{
    class KukaEkiIOInterface
    {
        private:
            std::string eki_server_address_;
            std::string eki_server_port_;
            int eki_cmd_buff_len_;
            int eki_max_cmd_buff_len_ = 5;
            int n_io_;

            int eki_read_state_timeout_ = 5;  // [s]; settable by parameter (default = 5)
            static void eki_handle_receive(const boost::system::error_code &ec, size_t length,
                                         boost::system::error_code* out_ec, size_t* out_length);
            boost::asio::io_service ios_;
            boost::asio::deadline_timer deadline_;
            boost::asio::ip::udp::endpoint eki_server_endpoint_;
            boost::asio::ip::udp::socket eki_server_socket_;
            void eki_check_read_state_deadline();

        public:
            KukaEkiIOInterface(const std::string& eki_server_address, const std::string& eki_server_port, int n_io);
            ~KukaEkiIOInterface(){};
            bool eki_read_state(std::vector<bool> &io_states, std::vector<int> &io_pins, std::vector<int> &io_types, int &cmd_buff_len);
            bool eki_write_command(const std::vector<int> &io_pins, const std::vector<int> &io_modes, const std::vector<bool> &target_ios);
    };
}
#endif