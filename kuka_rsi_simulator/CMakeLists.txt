cmake_minimum_required(VERSION 3.16)
project(kuka_rsi_simulator LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(kuka_rsi_simulator
  SCRIPTS_DESTINATION lib/kuka_rsi_simulator
)
ament_package()
