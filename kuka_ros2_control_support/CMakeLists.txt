cmake_minimum_required(VERSION 3.16)

project(kuka_ros2_control_support LANGUAGES CXX)

option(
  KUKA_ROS2_CONTROL_SUPPORT_BUILD_INTEGRATION_TESTS
  "Build integration tests"
  OFF
)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(ros2_control_node_steady_clock src/ros2_control_node_steady_clock.cpp)
ament_target_dependencies(ros2_control_node_steady_clock
  controller_manager
  rclcpp
)

install(
  TARGETS ros2_control_node_steady_clock
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)

  if(${KUKA_ROS2_CONTROL_SUPPORT_BUILD_INTEGRATION_TESTS})
    add_launch_test(test/ros2_control_support.py
      TIMEOUT
        180
    )
  endif()
endif()

ament_package()
