#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "robot_interfaces/msg/arm.hpp"
#include <hardware_interface/handle.hpp>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_interfaces/msg/arm.hpp>

namespace mycontrol {

class MyControl : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MyControl)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override {
        // 读取URDF中<ros2_control>的参数
        for (auto& joint : info.joints) {
            joint_names_.push_back(joint.name);
            state_positions_.push_back(0.0);
            state_velocities_.push_back(0.0);
            command_positions_.push_back(0.0);
        }
        node        = std::make_shared<rclcpp::Node>("control_node");
        publisher_  = node->create_publisher<robot_interfaces::msg::Arm>("myjoints_target", 10);
        subscriber_ = node->create_subscription<robot_interfaces::msg::Arm>("myjoints_state", 10, [this](const robot_interfaces::msg::Arm& msg) {
            for (int i = 0; i < 6; i++) {
                state_positions_[i]  = msg.joints[i].rad;
                state_velocities_[i] = msg.joints[i].omega;
            }
        });
        // TODO:初始化节点和订阅者，发布者，通过robot_driver与MCU交互
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        // 返回每个关节的位置/速度状态（按需构造，避免存储不可拷贝的接口对象）
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.reserve(joint_names_.size() * 2);
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            state_interfaces.emplace_back(joint_names_[i], "position", &state_positions_[i]);
            state_interfaces.emplace_back(joint_names_[i], "velocity", &state_velocities_[i]);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        // 返回每个关节的位置命令接口（按需构造）
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.reserve(joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            command_interfaces.emplace_back(joint_names_[i], "position", &command_positions_[i]);
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override {
        // 从MCU读取关节状态
        rclcpp::spin_some(node);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override {
        // 将期望位置/速度写入MCU
        robot_interfaces::msg::Arm msg;
        for (int i = 0; i < 6; i++) {
            msg.joints[i].rad    = (float)command_positions_[i];
            msg.joints[i].omega  = 0.0;
            msg.joints[i].torque = 0.0;
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

    std::vector<double> state_positions_;
    std::vector<double> state_velocities_;
    std::vector<double> command_positions_;
};

} // namespace mycontrol

// 注册插件
PLUGINLIB_EXPORT_CLASS(mycontrol::MyControl, hardware_interface::SystemInterface)