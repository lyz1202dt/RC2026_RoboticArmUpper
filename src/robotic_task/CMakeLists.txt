cmake_minimum_required(VERSION 3.8)
project(robotic_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)

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

add_executable(robotic_task
  src/main.cpp
  src/arm_handle_node.cpp
)

add_executable(arm_test
  src/test.cpp
)

target_include_directories(robotic_task
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/robotic_task>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robotic_task
  rclcpp
  rclcpp_action
  std_msgs
  builtin_interfaces
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_msgs
  robot_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)


#机械臂测试任务
ament_target_dependencies(arm_test
  rclcpp
  rclcpp_action
  std_msgs
  trajectory_msgs
  robot_interfaces
  geometry_msgs
)

install(TARGETS
robotic_task arm_test
DESTINATION lib/${PROJECT_NAME})

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
