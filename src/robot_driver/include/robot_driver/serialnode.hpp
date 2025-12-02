#ifndef __SERIALNODE_HPP__
#define __SERIALNODE_HPP__

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <cdc_trans.hpp>
#include <robot_interfaces/msg/arm.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <data_pack.h>


class SerialNode : public rclcpp::Node
{
public:
    SerialNode();
    ~SerialNode();

private:
    bool exit_thread;
    void legsSubscribCb(const robot_interfaces::msg::Arm &msg);
    void publishLegState(const Arm_t *arm_state);

    std::unique_ptr<CDCTrans> cdc_trans;
    std::unique_ptr<std::thread> usb_event_handle_thread;
    std::unique_ptr<std::thread> target_send_thread;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher;
    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr joint_subscriber;
    OnSetParametersCallbackHandle::SharedPtr param_server_handle;

    Arm_t arm_target;
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
};

#endif