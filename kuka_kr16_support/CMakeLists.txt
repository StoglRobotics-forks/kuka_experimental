cmake_minimum_required(VERSION 3.5)

project(kuka_kr16_support)

find_package(ament_cmake REQUIRED)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(test/roslaunch_test.xml)
endif()

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()