#include <memory>
#include <thread>

// ROS includes
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"

// code is inspired by
// https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // create executor
  std::shared_ptr<rclcpp::Executor> e =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // create controller manager instance
  rclcpp::NodeOptions node_options;
  // Required for getting types of controllers to be loaded via service call
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.clock_type(rcl_clock_type_t::RCL_STEADY_TIME);
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(
    e, "controller_manager", "", node_options);

  RCLCPP_WARN_STREAM(
    controller_manager->get_logger(),
    "Using a fixed period of:" << controller_manager->get_update_rate());

  // control loop thread
  std::thread control_loop(
    [controller_manager]()
    {
      // use fixed time step
      const rclcpp::Duration dt =
        rclcpp::Duration::from_seconds(1.0 / controller_manager->get_update_rate());

      while (rclcpp::ok())
      {
        controller_manager->read(controller_manager->now(), dt);
        controller_manager->update(controller_manager->now(), dt);
        controller_manager->write(controller_manager->now(), dt);
      }
    });

  // spin the executor with controller manager node
  e->add_node(controller_manager);
  e->spin();

  // wait for control loop to finish
  control_loop.join();

  // shutdown
  rclcpp::shutdown();

  return 0;
}