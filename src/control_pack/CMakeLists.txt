cmake_minimum_required(VERSION 3.8)
project(control_pack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

pluginlib_export_plugin_description_file(controller_interface mixcontroller_plugin.xml)

add_library(mycontrol SHARED
  src/mycontrol.cpp
)

add_library(mixcontroller SHARED
  src/mixcontroller.cpp
)

target_include_directories(mixcontroller PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

# target_link_libraries(mixcontroller
# PRIVATE
# rclcpp::rclcpp
# rclcpp_action::rclcpp_action
# )

ament_target_dependencies(
  mixcontroller
  rclcpp
  controller_interface
  joint_trajectory_controller
  pluginlib
  hardware_interface
)

ament_target_dependencies(mycontrol
  rclcpp
  hardware_interface
  pluginlib
  controller_manager_msgs
  urdf
  robot_interfaces
)

install(
  TARGETS mycontrol
  LIBRARY DESTINATION lib
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS mixcontroller
LIBRARY DESTINATION lib)

install(
  FILES mixcontroller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include)

ament_package()
