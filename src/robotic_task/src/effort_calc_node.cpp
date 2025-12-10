#include "effort_calc_node.hpp"
#include "std_msgs/msg/string.hpp"

EffortCalcNode::EffortCalcNode(const rclcpp::Node::SharedPtr& node) {

    this->node = node;
    publisher_ = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/my_joint_trajectory", 10
    );

    RCLCPP_INFO(node->get_logger(),"轨迹处理节点启动");

    //this->node->get_parameter("robot_description", urdf_xml);

    robo_desc_sub = node->create_subscription<std_msgs::msg::String>(
    "robot_description", 10,
    [this](const std_msgs::msg::String &msg) {
        urdf_xml = msg.data;
        RCLCPP_INFO(this->node->get_logger(),"订阅到消息2");
    });
    
    


    subscriber_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/robotic_arm_controller/joint_trajectory", 10, [this](const trajectory_msgs::msg::JointTrajectory& msg) {
            // TODO:计算力矩前馈值
            RCLCPP_INFO(this->node->get_logger(),"订阅到消息");

            if(first_run)
            {
                if(urdf_xml.empty())
                    return;
                first_run=false;
                kdl_parser::treeFromString(urdf_xml, tree);
                tree.getChain("base_link", "link6", chain);
            }

            // 1. 重力向量
            KDL::Vector gravity(0.0, 0.0, -9.81);
            // 2. KDL 动力学对象
            KDL::ChainDynParam dyn(chain, gravity);



            trajectory_msgs::msg::JointTrajectory target;
            target.joint_names     = msg.joint_names;
            target.header.frame_id = msg.header.frame_id;
            target.header.stamp    = this->node->get_clock()->now();

            target.points = msg.points;


            KDL::JntArray q(6), q_dot(6), q_ddot(6);

            // 4. KDL 动力学计算输出
            KDL::JntSpaceInertiaMatrix M_kdl(6);
            KDL::JntArray C_kdl(6), G_kdl(6);


            for (auto& point : target.points) {
                for(int i=0;i<6;i++)
                {
                    q(i)=point.positions[i];
                    q_dot(i)=point.velocities[i];
                    q_ddot(i)=point.accelerations[i];
                }

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
                    point.effort[i] = tau[i]; // 赋值给轨迹上的点
                }
                RCLCPP_INFO(this->node->get_logger(),"joint2_torque=%lf",point.effort[1]);
            }

            RCLCPP_INFO(this->node->get_logger(), "订阅到Moveit的下发轨迹，进行动力学计算并下发到ros2_control");
        }
    );
}
