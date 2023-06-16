cmake_minimum_required(VERSION 3.16)

project(kuka_ready2_educate_support LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()