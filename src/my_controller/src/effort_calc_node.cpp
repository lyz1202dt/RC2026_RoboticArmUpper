#include "effort_calc_node.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <memory>
#include <rclcpp/time.hpp>
#include <rclcpp_action/server.hpp>


EffortCalcNode::EffortCalcNode(const rclcpp::Node::SharedPtr node) {

    this->node = node;
    param_node = std::make_shared<rclcpp::Node>("param_node");
    // publisher_ = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    //     "/my_joint_trajectory", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile()
    // );

    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(param_node, "/robot_state_publisher");

    auto params = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml    = params[0].as_string();
    if(urdf_xml.empty())
    {
        RCLCPP_ERROR(node->get_logger(),"无法读取URDF文件，不能进行动力学计算");
    }

    kdl_parser::treeFromString(urdf_xml, tree);
    tree.getChain("base_link", "link6", chain);


    follow_joint_trajectory_server = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        node, "/robotic_arm_controller/arm_command", std::bind(&EffortCalcNode::server_goal_handle, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&EffortCalcNode::server_cancle_handle, this, std::placeholders::_1),
        std::bind(&EffortCalcNode::server_accepted_handle, this, std::placeholders::_1)
    );
    //trajectory_deal_thread=std::make_shared<std::thread>(std::bind(&EffortCalcNode::trajectory_handle,this));   //创建线程用于执行机械臂轨迹，并发送到ros2_control
}


rclcpp_action::GoalResponse EffortCalcNode::server_goal_handle(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
    RCLCPP_INFO(node->get_logger(),"接收到goal请求");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse EffortCalcNode::server_cancle_handle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
    RCLCPP_INFO(node->get_logger(),"取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
}


void EffortCalcNode::server_accepted_handle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
    RCLCPP_INFO(node->get_logger(),"goal接受后执行");
}

void EffortCalcNode::trajectory_handle()
{
    while(1)
    {
        
    }
}

