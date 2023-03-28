cmake_minimum_required(VERSION 3.16)

project(kuka_ros2_control_sim_support LANGUAGES CXX)

option(
  KUKA_ROS2_CONTROL_SIM_SUPPORT_BUILD_INTEGRATION_TESTS
  "Build integration tests"
  ON
)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)

  if(${KUKA_ROS2_CONTROL_SIM_SUPPORT_BUILD_INTEGRATION_TESTS})
    add_launch_test(test/ros2_control_sim_support.py
      TIMEOUT
        180
    )
  endif()
endif()
