#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include "arm_handle_node.hpp"

namespace {

std::string LoadTextFile(const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        return "";
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // 默认启用仿真时钟并将MoveIt状态输入绑定到ros2_control发布的机械臂关节状态，
    // 避免 wall-time 与 sim-time 混用导致 current_state_monitor 判定状态过期。
    rclcpp::NodeOptions node_options;
    node_options.arguments({
        "--ros-args",
        "-p", "use_sim_time:=true",
        "-r", "joint_states:=/joint_state_broadcaster/joint_states"
    });
    auto node=std::make_shared<rclcpp::Node>("arm_task_handle_node", node_options);

    // 直接运行 robotic_task 时，MoveGroupInterface 需要本节点具备 robot_description 与 SRDF 参数。
    node->declare_parameter<std::string>("robot_description", "");
    node->declare_parameter<std::string>("robot_description_semantic", "");

    std::string urdf_xml = node->get_parameter("robot_description").as_string();
    if (urdf_xml.empty()) {
        const std::string robotic_arm_share = ament_index_cpp::get_package_share_directory("robotic_arm");
        urdf_xml = LoadTextFile(robotic_arm_share + "/urdf/robotic_arm_mujoco.urdf");
        if (!urdf_xml.empty()) {
            node->set_parameter(rclcpp::Parameter("robot_description", urdf_xml));
        } else {
            RCLCPP_WARN(node->get_logger(), "robot_description 为空且本地URDF加载失败，后续可能无法初始化 MoveGroupInterface");
        }
    }

    std::string srdf_xml = node->get_parameter("robot_description_semantic").as_string();
    if (srdf_xml.empty()) {
        const std::string robotic_config_share = ament_index_cpp::get_package_share_directory("robotic_config");
        srdf_xml = LoadTextFile(robotic_config_share + "/config/robotic_arm.srdf");
        if (!srdf_xml.empty()) {
            node->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf_xml));
        } else {
            RCLCPP_WARN(node->get_logger(), "robot_description_semantic 为空且本地SRDF加载失败，后续可能无法初始化 MoveGroupInterface");
        }
    }

    auto arm_handle=std::make_shared<ArmHandleNode>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/*
上位机代码结构构想：
节点：robotic_task(一进入main函数，就无限永久spin)
    Action服务端：arm_handle_action
    Topic发布者：joint_target

    如果有订阅到动作请求，那么执行几次抓取规划放到队列中，然后让一个线程去操作Moveit的接口，解析plan和发布joint_target，
    最后抓取成功后返回SUCCESS，如果规划失败，返回REJECT，表示需要重新移动机器人位置方便机械臂能够抓取到。
    确认抓取成功后，自己再规划一次，将期望目标点放到队列中，根据当前机器人的状态（存储一个块，存储两个块）【状态保存】，确定机械臂要到达的位置和是否放下块.

节点：robot_driver(一进入main函数，就无限永久spin)
    Topic订阅者：myjoints_target    订阅电机目标，发送给MCU
    Topic发布者：joint_states       向Moveit发布关节的状态

节点（李振宇）：vision_search
    Action客户端 arm_handle_action:请求抓取一次物块
    Action服务端 get_kfs_action：接受robot_run节点的请求获取KFS的服务
    //其他部分

    任务：
    识别KFS，查询相机坐标系到机械臂base_link的TF变换，将坐标变换为base_link下的坐标后发送给arm_handle_action，
    并监听反馈，再将反馈继续传递给robot_run，如果直接识别失败也要通知robot_run

节点（张坤）：robot_run
    Action客户端get_kfs_action  :在特定时刻请求一次获取KFS，失败后尝试移动机器人再次尝试
    //其他部分
    规划机器人总体运动，目前先用Action写请求vision_search节点进行识别。
*/
