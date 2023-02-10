cmake_minimum_required(VERSION 3.5)
project(example_6dof_position_only_system)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake 
  hardware_interface 
  kuka_rsi_hw_interface
  pluginlib 
  rclcpp 
)

# find dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
#find_package(tinyxml REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(
	${PROJECT_NAME}
	SHARED
	src/kuka_system_position_only.cpp
)
target_include_directories(
	${PROJECT_NAME}
	PRIVATE
	include
)

target_link_libraries(
	${PROJECT_NAME}
	tinyxml
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface
										 ros2_control_kuka_driver.xml)


install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config description launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
