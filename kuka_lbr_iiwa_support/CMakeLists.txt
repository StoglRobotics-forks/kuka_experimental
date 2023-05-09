cmake_minimum_required(VERSION 3.16)

project(kuka_lbr_iiwa_support LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# temporarily: until sources are moved to other org/rep
install(
  DIRECTORY
    meshes/lbr_iiwa_14_r820/collision
  DESTINATION
  share/${PROJECT_NAME}/meshes/lbr_iiwa_14_r820/
)

# temporarily: until sources are moved to other org/rep
install(
  DIRECTORY
    meshes/lbr_iiwa_14_r820/visual
  DESTINATION
  share/${PROJECT_NAME}/meshes/lbr_iiwa_14_r820/
)

ament_package()
