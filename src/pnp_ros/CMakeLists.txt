cmake_minimum_required(VERSION 3.8)
project(pnp_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)

add_executable(pnp_ros_node src/pnp_ros.cpp)
target_include_directories(pnp_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(pnp_ros_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  pnp_ros_node
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "sensor_msgs"
  "cv_bridge"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "robot_interfaces"
)
target_link_libraries(pnp_ros_node ${OpenCV_LIBS})
target_link_libraries(pnp_ros_node CURL::libcurl)

install(TARGETS pnp_ros_node
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
