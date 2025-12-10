#include "robot_interfaces/msg/arm.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <rclcpp/parameter.hpp>
#include <rclcpp/time.hpp>
#include <serialnode.hpp>
#include <chrono>

using namespace std::chrono_literals;

SerialNode::SerialNode()
    : Node("driver_node") {

    arm_target.pack_type=1;

    this->declare_parameter("enable_air_pump",false);   //使能气泵
    param_server_handle=this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &param){
        rcl_interfaces::msg::SetParametersResult result;
        (void)param;
        result.successful=true;
        RCLCPP_INFO(this->get_logger(),"参数更新");
        return result;
    });

    // 初始化状态
    exit_thread = false;

    // 先创建 publisher/subscriber，确保回调中 publish 时 publisher 已就绪
    //joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    joint_publisher = this->create_publisher<robot_interfaces::msg::Arm>("myjoints_state", 10);
    
    joint_subscriber = this->create_subscription<robot_interfaces::msg::Arm>(
        "myjoints_target", 10, std::bind(&SerialNode::legsSubscribCb, this, std::placeholders::_1));


    cdc_trans = std::make_unique<CDCTrans>();                           // 创建CDC传输对象
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) { // 注册接收回调
        //RCLCPP_INFO(this->get_logger(), "接收到了数据包,长度%d", size);
        if (size == sizeof(Arm_t)) // 验证包长度，可以被视作四条腿的状态数据包
        {
            const Arm_t* pack = reinterpret_cast<const Arm_t*>(data);
            if (pack->pack_type == 1)  // 确认包类型正确
                publishLegState(pack);
        }
    });
    if(!cdc_trans->open(0x0483, 0x5740))                                // 开启USB_CDC传输接口
    {
        RCLCPP_ERROR(get_logger(),"串口打开失败，无法驱动物理机械臂！");
        exit_thread=true;
    }

    // 创建线程处理CDC消息（在 open 之后、publisher 创建之后）
    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        do{
            cdc_trans->process_once();
        }while (!exit_thread);
    });


    target_send_thread=std::make_unique<std::thread>([this](){
        do{
            auto now = std::chrono::system_clock::now();
            cdc_trans->send_struct(arm_target);
            std::this_thread::sleep_until(now + 10ms);
        }while (!exit_thread);
    });
}

SerialNode::~SerialNode() {
    // 请求线程退出并等待其结束，保证安全关闭
    exit_thread = true;
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }
    if(target_send_thread&&target_send_thread->joinable()){
        target_send_thread->join();
    }
    if (cdc_trans) {
        cdc_trans->close();
    }
}

void SerialNode::publishLegState(const Arm_t* arm_state) {
    /*sensor_msgs::msg::JointState msg;
    msg.header.stamp=this->now();
    msg.name={"joint1","joint2","joint3","joint4","joint5","joint6"};       //发布joint状态
    msg.position.resize(6);
    msg.velocity.resize(6);
    for(int i=0;i<6;i++)
    {
        msg.position[i]=arm_state->joints[i].rad;
        msg.velocity[i]=arm_state->joints[i].omega;
    }*/
    robot_interfaces::msg::Arm msg;
    for(int i=0;i<6;i++)
    {
        msg.joints[i].rad=arm_state->joints[i].rad;
        msg.joints[i].omega=arm_state->joints[i].omega;
        msg.joints[i].torque=arm_state->joints[i].torque;
    }
    msg.joints[5].rad=arm_target.joints[5].rad; //欺骗Moveit认为不存在的关节6到达目标位置
    joint_publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(), "发布电机状态");
}

void SerialNode::legsSubscribCb(const robot_interfaces::msg::Arm& msg) {
    for (int i = 0; i < 6; i++) {
        arm_target.joints[i].rad=msg.joints[i].rad;
        arm_target.joints[i].omega=msg.joints[i].omega;
        arm_target.joints[i].torque=msg.joints[i].torque;
    }
    //cdc_trans->send_struct(legs_target); // 一旦订阅到最新的包，立即发送到下位机（下位机用定时器保证匀速率发送方便断开检测）
    RCLCPP_INFO(this->get_logger(), "订阅到电机目标值,torque2=%lf",msg.joints[1].torque);
}
