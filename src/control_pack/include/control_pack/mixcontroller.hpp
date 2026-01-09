#pragma once

#include <Eigen/Dense>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <joint_trajectory_controller/interpolation_methods.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace mixcontroller {


// 五次多项式轨迹插值算法
class QuinticParam {
public:
    void set_param(
        const double t0, const double t1, const double p0, const double v0, const double a0, const double pt, const double v1, const double at
    );
    double get_pos(const double t);
    double get_vel(const double t);
    double get_acc(const double t);

private:
    double a{0.0};
    double b{0.0};
    double c{0.0};
    double d{0.0};
    double e{0.0};
    double f{0.0};
    double t0{0.0};
    double t1{0.0};
};


// 管理多段连续轨迹的执行
class ContinuousTrajectory {
public:
    explicit ContinuousTrajectory();
    bool get_target(const rclcpp::Time& time, trajectory_msgs::msg::JointTrajectoryPoint& output); // 获取到时间x处的轨迹
    void start_track(rclcpp::Time now);                                                                            // 开始进行轨迹跟踪
    void set_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);                  // 设置原始轨迹
private:
    QuinticParam line[6];
    size_t cur_index; // points 容器的大小
    rclcpp::Time start_time;
    trajectory_msgs::msg::JointTrajectory trajectory;   // 当前要执行的轨迹
};


class MixController : public controller_interface::ControllerInterface {
public:
    MixController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> activate_goal_handle_;
    control_msgs::action::FollowJointTrajectory::Feedback::SharedPtr feedback_msg;
    control_msgs::action::FollowJointTrajectory::Result::SharedPtr result_msg;
    trajectory_msgs::msg::JointTrajectory current_trajectory_;

    trajectory_msgs::msg::JointTrajectoryPoint output_state;

    std::vector<std::string> joint_names_;

    ContinuousTrajectory continue_trajectory;

    KDL::Tree tree;
    KDL::Chain chain;
    std::string urdf_xml;
    rclcpp::Node::SharedPtr param_node;
    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;

    // 动力学参数计算
    KDL::Vector gravity; // 重力向量
    std::shared_ptr<KDL::ChainDynParam> dyn; // 动力学计算对象
    KDL::JntSpaceInertiaMatrix M_kdl; //  惯性矩阵
    KDL::JntArray C_kdl, G_kdl; // 科里奥利力和重力
    KDL::JntArray q_kdl, dq_kdl, ddq_kdl; // 科里奥利力和重力


    rclcpp_action::GoalResponse
        handle_goal(const rclcpp_action::GoalUUID& uuid, const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse
        handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

    bool is_execut_trajectory{false};
    bool cancle_execut{false};
    bool finished_execut{false};


    Eigen::Vector<double, 6> dynamicCalc();
};


} // namespace mixcontroller