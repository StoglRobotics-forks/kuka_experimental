#ifndef KUKA_RSI_HW_INTERFACE__KUKA_SYSTEM_POSITION_HPP_
#define KUKA_RSI_HW_INTERFACE__KUKA_SYSTEM_POSITION_HPP_

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
class KukaSystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaSystemPositionOnlyHardware);

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

#endif  // KUKA_RSI_HW_INTERFACE__KUKA_SYSTEM_POSITION_HPP_
