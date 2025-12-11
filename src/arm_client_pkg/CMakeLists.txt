cmake_minimum_required(VERSION 3.8)
project(arm_client_pkg)

# 编译为 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)  # 自定义 action/msg 包
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 查找系统依赖
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# 包含头文件路径
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(point_client 
  src/point_client.cpp
   src/inference.cpp)

# 链接依赖库
ament_target_dependencies(point_client
  rclcpp
  rclcpp_action
  geometry_msgs
  robot_interfaces
  tf2
  tf2_geometry_msgs
)

target_link_libraries(point_client
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
)

# 安装可执行文件
install(TARGETS point_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
