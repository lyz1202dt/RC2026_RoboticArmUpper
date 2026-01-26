/*
实现控制器与电机硬件之间的连接通信
*/

#include "hardware_interface/system_interface.hpp" // 硬件接口基类
#include "pluginlib/class_list_macros.hpp"  // 插件库宏

// #include "robot_interfaces/msg/arm.hpp"
#include <hardware_interface/handle.hpp> // 硬件接口句柄
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_interfaces/msg/arm.hpp>

// 连接控制器和电机硬件

namespace mycontrol {

class MyControl : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MyControl)

    // 初始化
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override {
        // hardware_interface::HardwareInfo 关节列表
        // 读取 URDF 中 <ros2_control> 的参数，从 urdf 读取关节配置
        for (auto& joint : info.joints) {
            joint_names_.push_back(joint.name); // 保存关节名

            // 初始化状态和命令向量
            state_positions_.push_back(0.0); // 实际位置
            state_velocities_.push_back(0.0); // 实际速度
            command_positions_.push_back(0.0); // 目标位置
            command_effort_.push_back(0.0); // 目标力矩
            command_velocity_.push_back(0.0); // 目标速度
        }

        // 创建 ros2 节点
        node        = std::make_shared<rclcpp::Node>("control_node");

        // 创建发布者（发送命令给 MCU）
        publisher_  = node->create_publisher<robot_interfaces::msg::Arm>("myjoints_target", 10);

        // 创建订阅者（接收 MCU 返回的状态）
        subscriber_ = node->create_subscription<robot_interfaces::msg::Arm>("myjoints_state", 10, [this](const robot_interfaces::msg::Arm& msg) {
            for (int i = 0; i < 6; i++) {
                state_positions_[i]  = msg.joints[i].rad;
                state_velocities_[i] = msg.joints[i].omega;
            }
        });
        // TODO:初始化节点和订阅者，发布者，通过robot_driver与MCU交互

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // 导出状态接口
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        // 返回每个关节的位置/速度状态（按需构造，避免存储不可拷贝的接口对象）
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.reserve(joint_names_.size() * 2);
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            // 导出位置状态接口
            // emplace_back 在向量的末尾直接构造元素，不同于 push_bask 的先构造再拷贝
            state_interfaces.emplace_back(joint_names_[i], "position", &state_positions_[i]);
            // 导出速度状态接口
            state_interfaces.emplace_back(joint_names_[i], "velocity", &state_velocities_[i]);
        }
        return state_interfaces;
    }

    // 导出命令接口
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        // 返回每个关节的位置命令接口（按需构造）
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.reserve(joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            command_interfaces.emplace_back(joint_names_[i], "position", &command_positions_[i]);
            command_interfaces.emplace_back(joint_names_[i], "velocity", &command_velocity_[i]);
            command_interfaces.emplace_back(joint_names_[i], "effort", &command_effort_[i]);
        }
        return command_interfaces;
    }

    // 激活
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // 停用
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // 读取状态
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override {
        // 从MCU读取关节状态
        rclcpp::spin_some(node);
        return hardware_interface::return_type::OK;
    }

    // 发送命令
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override {
        // 将期望位置/速度写入MCU
        // 将命令发送给 MCU
        robot_interfaces::msg::Arm msg;
        for (int i = 0; i < 6; i++) {
            msg.joints[i].rad    = (float)command_positions_[i];
            msg.joints[i].omega  = (float)command_velocity_[i];
            msg.joints[i].torque = (float)command_effort_[i];
        }
        publisher_->publish(msg);
        rclcpp::spin_some(node);
        return hardware_interface::return_type::OK;
    }

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr subscriber_;
    rclcpp::Publisher<robot_interfaces::msg::Arm>::SharedPtr publisher_;

    std::vector<std::string> joint_names_;

    //控制接口
    std::vector<double> state_positions_;
    std::vector<double> state_velocities_;
    std::vector<double> command_positions_;
    std::vector<double> command_velocity_;
    std::vector<double> command_effort_;
};

} // namespace mycontrol

// 注册插件
PLUGINLIB_EXPORT_CLASS(mycontrol::MyControl, hardware_interface::SystemInterface)