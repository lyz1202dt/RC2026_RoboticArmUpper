#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "vision_search/srv/set_point.hpp"
#include <termios.h>
#include <unistd.h>

using SetPoint = vision_search::srv::SetPoint;

// ----------- 键盘输入工具函数（检测键盘按键） -----------
int getKey()
{
    struct termios oldt, newt;
    int ch;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

// -------------------- 客户端节点 --------------------
class PointClientNode : public rclcpp::Node
{
public:
    PointClientNode() : Node("point_client_node")
    {
        client_ = this->create_client<SetPoint>("server_point");

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for service...");
        }
        RCLCPP_INFO(this->get_logger(), "Service available. Press R to request point.");
    }

    void call_service()
    {
        auto request = std::make_shared<SetPoint::Request>();

        auto future = client_->async_send_request(request);

        RCLCPP_INFO(this->get_logger(), "Request sent, waiting for response...");

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto resp = future.get();

            if (resp->success) {
                RCLCPP_INFO(this->get_logger(),
                            "\nReceived point:\n X= %.3f  Y=%.3f  Z=%.3f \n roll=%.3f  pitch=%.3f  yaw=%.3f  \n state = %s  msg=%s",
                            resp->x, resp->y, resp->z, resp->roll,resp->pitch,resp->yaw,
                            resp->success ? "true" : "false",
                            resp->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Service replied with failure: %s",
                            resp->message.c_str());
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

private:
    rclcpp::Client<SetPoint>::SharedPtr client_;
};


// -------------------- main --------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PointClientNode>();

    while (rclcpp::ok())
    {
        int c = getKey();

        if (c == 'r' || c == 'R') {
            node->call_service();
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
