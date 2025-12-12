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

KDL::JntArray EffortCalcNode::DynamicCalc(const KDL::JntArray& q, const KDL::JntArray& q_dot, const KDL::JntArray& q_ddot) {
    KDL::JntArray effort(6);
    // 1. 重力向量
    KDL::Vector gravity(0.0, 0.0, -9.81);
    // 2. KDL 动力学对象
    KDL::ChainDynParam dyn(chain, gravity);


    // trajectory_ff.header.frame_id=msg->header.frame_id;
    // trajectory_ff.header.stamp=this->node->get_clock()->now();
    // trajectory_ff.joint_names=msg->joint_names;
    // trajectory_ff.points=msg->points;

    // 4. KDL 动力学计算输出
    KDL::JntSpaceInertiaMatrix M_kdl(6);
    KDL::JntArray C_kdl(6), G_kdl(6);

    // 5. 调用 KDL 动力学函数
    dyn.JntToMass(q, M_kdl);
    dyn.JntToCoriolis(q, q_dot, C_kdl);
    dyn.JntToGravity(q, G_kdl);

    // 6. 转换 KDL 输出到 Eigen，方便矩阵运算
    Eigen::Matrix<double, 6, 6> M_mat;
    Eigen::Matrix<double, 6, 1> C, G, qddot;

    for (int i = 0; i < 6; ++i) {
        C(i)     = C_kdl(i);
        G(i)     = G_kdl(i);
        qddot(i) = q_ddot(i);
        for (int j = 0; j < 6; ++j) {
            M_mat(i, j) = M_kdl(i, j);
        }
    }

    // 7. 计算前馈力矩 tau
    auto tau = M_mat * qddot + C + G;

    for (int i = 0; i < 6; i++) {
        effort(i) = tau[i]; // 赋值给轨迹上的点
    }
    return effort;
}
