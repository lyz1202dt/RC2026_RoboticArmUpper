#ifndef __SERIALNODE_HPP__
#define __SERIALNODE_HPP__

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <cdc_trans.hpp>
#include <robot_interfaces/msg/arm.hpp>
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
    Arm_t arm_target;
    rclcpp::Publisher<robot_interfaces::msg::Arm>::SharedPtr robot_pub;
    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr robot_sub;
};

#endif