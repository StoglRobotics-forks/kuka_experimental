cmake_minimum_required(VERSION 3.16)
project(kuka_rsi_hw_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

option(
  KUKA_ROS2_CONTROL_SUPPORT_BUILD_INTEGRATION_TESTS
  "Build integration tests"
  OFF
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

add_library(kuka_rsi_hw_interface SHARED
  src/kuka_system_position_only.cpp
)
target_compile_features(kuka_rsi_hw_interface PUBLIC cxx_std_17)
target_include_directories(kuka_rsi_hw_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/kuka_rsi_hw_interface>
)
# target_link_libraries(kuka_rsi_hw_interface
#   tinyxml
# )
ament_target_dependencies(kuka_rsi_hw_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})


pluginlib_export_plugin_description_file(hardware_interface kuka_rsi_hw_interface.xml)


if(BUILD_TESTING)
  # find_package(launch_testing_ament_cmake)

  # if(${KUKA_ROS2_CONTROL_SUPPORT_BUILD_INTEGRATION_TESTS})
  #   add_launch_test(test/test_rsi_hw_interface.py
  #     TIMEOUT
  #       180
  #   )
  # endif()
endif()

install(
  DIRECTORY include/
  DESTINATION include/kuka_rsi_hw_interface
)

install(
  TARGETS kuka_rsi_hw_interface
  EXPORT export_kuka_rsi_hw_interface
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY config krl
  DESTINATION share/kuka_rsi_hw_interface
)

ament_export_targets(export_kuka_rsi_hw_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
