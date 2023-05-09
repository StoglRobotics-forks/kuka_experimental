cmake_minimum_required(VERSION 3.16)
project(kuka_ros2_control_support LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  rclcpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(ros2_control_node_steady_clock src/ros2_control_node_steady_clock.cpp)
target_include_directories(ros2_control_node_steady_clock PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/kuka_ros2_control_support>
)
ament_target_dependencies(ros2_control_node_steady_clock PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)

  add_launch_test(test/ros2_control_support.py TIMEOUT 180)

endif()

install(
  TARGETS ros2_control_node_steady_clock
  RUNTIME DESTINATION lib/kuka_ros2_control_support
)

install(
  DIRECTORY config urdf launch
  DESTINATION share/kuka_ros2_control_support
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
