#include <rclcpp/rclcpp.hpp>
#include "effort_calc_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node=std::make_shared<rclcpp::Node>("arm_task_handle_node");
    auto arm_handle=std::make_shared<EffortCalcNode>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
