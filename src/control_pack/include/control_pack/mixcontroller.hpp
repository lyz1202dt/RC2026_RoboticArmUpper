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

/* =====================================================================
 *                      一、五次多项式参数类
 * ===================================================================== */

/**
 * @brief 单关节五次多项式轨迹参数类
 *
 * 作用：
 *   对“单个关节”在 [t0, t1] 时间段内的运动进行建模
 *
 * 边界条件（共 6 个）：
 *   t = t0 : p0, v0, a0
 *   t = t1 : pt, v1, at
 *
 * 五次多项式正好有 6 个未知数，可以严格满足这些条件
 *
 * 多项式形式：
 *   p(t) = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f
 *
 * 连续性：
 *   C0：位置连续
 *   C1：速度连续
 *   C2：加速度连续
 *
 */
class QuinticParam {
public:
    /**
     * @brief 设置多项式参数（一次性解出 a~f）
     *
     * @param t0 起始时间
     * @param t1 终止时间
     * @param p0 起始位置
     * @param v0 起始速度
     * @param a0 起始加速度
     * @param pt 目标位置
     * @param v1 目标速度
     * @param at 目标加速度
     */
    void set_param(
        const double t0, const double t1, const double p0, const double v0, 
        const double a0, const double pt, const double v1, const double at
    );

    /// 给定时间 t，计算位置
    double get_pos(const double t);

    /// 给定时间 t，计算速度
    double get_vel(const double t);

    /// 给定时间 t，计算加速度
    double get_acc(const double t);

private:
    /* ===== 五次多项式系数 ===== */
    double a{0.0};
    double b{0.0};
    double c{0.0};
    double d{0.0};
    double e{0.0};
    double f{0.0};

    /* ===== 当前段的时间范围 ===== */
    double t0{0.0};
    double t1{0.0};
};


/* =====================================================================
 *                      二、连续轨迹管理类
 * ===================================================================== */

/**
 * @brief 连续轨迹管理类
 *
 * 功能职责：
 *  1. 保存一条 JointTrajectory（来自 action goal）
 *  2. 在相邻 trajectory point 之间：
 *       - 对每个关节建立一个 QuinticParam
 *       - 实现平滑插值
 *  3. 给定当前时间 time，输出期望：
 *       - position
 *       - velocity
 *       - acceleration
 *
 * 本质：
 *   JointTrajectory  →  连续时间函数 q(t), dq(t), ddq(t)
 */
class ContinuousTrajectory {
public:
    explicit ContinuousTrajectory();

    /**
     * @brief 根据当前时间计算目标轨迹点
     *
     * @param time 当前控制周期的 ROS 时间
     * @param output 输出的目标关节状态（pos/vel/acc）
     * @return 是否成功获取
     */
    bool get_target(const rclcpp::Time& time, trajectory_msgs::msg::JointTrajectoryPoint& output); // 获取到时间x处的轨迹
    
    /**
     * @brief 开始执行轨迹（记录起始时间）
     */
    void start_track(rclcpp::Time now);                                                                            // 开始进行轨迹跟踪
    
    /**
     * @brief 设置一整条新的关节轨迹
     *
     * 通常在接收到 FollowJointTrajectory goal 后调用
     */
    void set_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);                  // 设置原始轨迹

private:
    /**
     * @brief 每个关节一条五次多项式（这里假设 6 关节）
     *
     * line[i] 表示：
     *   第 i 个关节在当前 trajectory segment 内的插值函数
     */
    QuinticParam line[6];

    /// 当前执行到 trajectory.points 的索引
    size_t cur_index; // points 容器的大小

    /// 当前轨迹的起始时间（time = now - start_time）
    rclcpp::Time start_time;

    /// 当前正在执行的 JointTrajectory
    trajectory_msgs::msg::JointTrajectory trajectory;   // 当前要执行的轨迹
};


/* =====================================================================
 *                      三、MixController 控制器
 * ===================================================================== */

/**
 * @brief 自定义 ros2_control 控制器
 *
 * 特点：
 *  - 支持 FollowJointTrajectory action
 *  - 内部使用五次多项式进行插值
 *  - 结合 KDL 进行动力学计算（M, C, G）
 *
 * 典型调用流程：
 *  on_init →
 *  on_configure →
 *  on_activate →
 *  update() [周期调用]
 */
class MixController : public controller_interface::ControllerInterface {
public:
    MixController();

    /* ================= 生命周期接口 ================= */
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief 控制器主循环（硬实时）
     *
     * 典型内容：
     *   - 读取当前关节状态
     *   - 计算期望轨迹
     *   - 计算控制量（如 torque）
     *   - 写入 command interface
     */
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    /* ================= 接口配置 ================= */
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    /* ================= FollowJointTrajectory Action ================= */

    /// action server
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_server_;
    
    /// 当前激活的 goal
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> activate_goal_handle_;
    
    /// 实时反馈给 action 客户端
    control_msgs::action::FollowJointTrajectory::Feedback::SharedPtr feedback_msg;

    /// 最终 action 结果
    control_msgs::action::FollowJointTrajectory::Result::SharedPtr result_msg;

    /// 当前 action 对应的完整轨迹
    trajectory_msgs::msg::JointTrajectory current_trajectory_;

    /// 控制周期中计算得到的目标点
    trajectory_msgs::msg::JointTrajectoryPoint output_state;

    /// 关节名称列表（顺序非常重要）
    std::vector<std::string> joint_names_;

    /// 连续轨迹管理器（核心插值模块）
    ContinuousTrajectory continue_trajectory; // 轨迹管理对象：负责存储轨迹、计算插值


    /* ================= KDL 模型 ================= */

    /// URDF → KDL Tree
    KDL::Tree tree;

    /// 从 base 到 end-effector 的关节链
    KDL::Chain chain;

    /// URDF XML 字符串
    std::string urdf_xml;

    /// 参数节点（用于读取 robot_description）
    rclcpp::Node::SharedPtr param_node;

    /// 参数客户端（从 robot_state_publisher 读取 URDF）
    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;


    /* ================= 动力学计算 ================= */

    /// 重力向量（一般 [0, 0, -9.81]）
    KDL::Vector gravity; 

    /// 动力学求解器（用于 M, C, G）
    std::shared_ptr<KDL::ChainDynParam> dyn; 

    /// 惯性矩阵 M(q)
    KDL::JntSpaceInertiaMatrix M_kdl; 

    /// 科里奥利项 C(q, dq) & 重力项 G(q)
    KDL::JntArray C_kdl, G_kdl; 

    /// 当前关节状态（q, dq, ddq）
    KDL::JntArray q_kdl, dq_kdl, ddq_kdl; 


    /* ================= Action 回调 ================= */

    rclcpp_action::GoalResponse
        handle_goal(const rclcpp_action::GoalUUID& uuid, const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse
        handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);


    /* ================= 执行状态标志 ================= */
    bool is_execut_trajectory{false};
    bool cancle_execut{false};
    bool finished_execut{false};

    /**
     * @brief 动力学计算接口
     *
     * @return 计算得到的控制量（如关节力矩）
     */
    Eigen::Vector<double, 6> dynamicCalc();
};


} // namespace mixcontroller