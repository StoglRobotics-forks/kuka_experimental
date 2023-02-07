cmake_minimum_required(VERSION 3.16)

project(kuka_kr5_support LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(launch_testing_ament_cmake)

add_launch_test(test/ros_launch_test.py)


install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
  )

  ament_package()
