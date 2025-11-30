cmake_minimum_required(VERSION 3.8)
project(vision_search)

# 默认编译标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------
# 找依赖
# ---------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# ---------------------
# 自定义库
# ---------------------
add_library(inference STATIC src/inference.cpp)
target_include_directories(inference
    PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
)

# ---------------------
# 生成自定义服务
# ---------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPoint.srv"
)

# ---------------------
# 可执行文件
# ---------------------
add_executable(point_server src/point_server.cpp)
add_executable(point_client src/point_client.cpp)

# ---------------------
# 连接 ROS 2 依赖
# ---------------------
ament_target_dependencies(point_server
  rclcpp
  rosidl_default_runtime
)

ament_target_dependencies(point_client
  rclcpp
  rosidl_default_runtime
)

# ---------------------
# 连接第三方库
# ---------------------
target_link_libraries(point_server
  inference
  ${OpenCV_LIBS}
  realsense2
)

target_link_libraries(point_client
  inference
  ${OpenCV_LIBS}
  realsense2
)

# ---------------------
# 将生成的服务接口连接到可执行文件
# ---------------------
rosidl_target_interfaces(point_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(point_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---------------------
# 安装可执行文件
# ---------------------
install(TARGETS
  point_server
  point_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
