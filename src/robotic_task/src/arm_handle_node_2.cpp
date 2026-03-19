#include "arm_handle_node.hpp"
// #include "GraspOrientionController.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cassert>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/detail/constraints__struct.hpp>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
// #include <qt5/QtGui/qvalidator.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <thread>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>

using namespace std::chrono_literals;


