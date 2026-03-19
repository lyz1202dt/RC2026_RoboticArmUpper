#include "control_pack/realtime_ik_solver.hpp"
#include <cmath>

namespace mixcontroller {

RealtimeIKSolver::RealtimeIKSolver(const KDL::Chain& chain, rclcpp::Logger logger)
    : chain_(chain), jacobian_(chain.getNrOfJoints()), logger_(logger) {
    jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    max_velocities_.resize(chain_.getNrOfJoints(), 2.0);  // 默认最大速度 2 rad/s
    RCLCPP_INFO(logger_, "RealtimeIKSolver 初始化完成，自由度: %zu", chain_.getNrOfJoints());
}

bool RealtimeIKSolver::solveTwistToJointVelocity(
    const geometry_msgs::msg::Twist& twist,
    const KDL::JntArray& current_q,
    KDL::JntArray& joint_velocities,
    double damping_factor) {
    
    // 1. 计算当前位置的 Jacobian 矩阵
    if (jacobian_solver_->JntToJac(current_q, jacobian_) != KDL::SolverI::E_NOERROR) {
        RCLCPP_WARN(logger_, "Jacobian 计算失败");
        return false;
    }

    // 2. 将 Twist 转换为 6 维向量
    Eigen::Vector<double, 6> twist_vec;
    twist_vec(0) = twist.linear.x;
    twist_vec(1) = twist.linear.y;
    twist_vec(2) = twist.linear.z;
    twist_vec(3) = twist.angular.x;
    twist_vec(4) = twist.angular.y;
    twist_vec(5) = twist.angular.z;

    // 3. 转换 KDL Jacobian 到 Eigen 矩阵
    Eigen::MatrixXd J(6, chain_.getNrOfJoints());
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < chain_.getNrOfJoints(); ++j) {
            J(i, j) = jacobian_(i, j);
        }
    }

    // 4. 计算阻尼伪逆
    Eigen::MatrixXd J_inv = compute_damped_pseudoinverse(J, damping_factor);

    // 5. 计算关节速度：q_dot = J^# * v_ee
    Eigen::VectorXd qd_eigen = J_inv * twist_vec;

    // 6. 应用关节速度限制
    std::vector<double> qd_vec(qd_eigen.data(), qd_eigen.data() + qd_eigen.size());
    apply_velocity_limits(qd_vec);

    // 7. 转换回 KDL JntArray
    joint_velocities.resize(chain_.getNrOfJoints());
    for (size_t i = 0; i < chain_.getNrOfJoints(); ++i) {
        joint_velocities(i) = qd_vec[i];
    }

    return true;
}

Eigen::MatrixXd RealtimeIKSolver::compute_damped_pseudoinverse(
    const Eigen::MatrixXd& J,
    double damping_factor) {
    
    // 使用 SVD 计算伪逆：J^# = J^T (J J^T + λ^2 I)^{-1}
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    const Eigen::MatrixXd& U = svd.matrixU();
    const Eigen::MatrixXd& V = svd.matrixV();
    const Eigen::VectorXd& singular_values = svd.singularValues();
    
    // 计算阻尼伪逆：对每个奇异值进行阻尼处理
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> S_inv(J.cols());
    int min_dim = std::min((int)J.rows(), (int)J.cols());
    for (int i = 0; i < min_dim; ++i) {
        double sigma = singular_values(i);
        // DLS：σ* = σ / (σ^2 + λ^2)
        S_inv.diagonal()(i) = sigma / (sigma * sigma + damping_factor * damping_factor);
    }
    
    return V * S_inv * U.transpose();
}

void RealtimeIKSolver::apply_velocity_limits(std::vector<double>& qd) {
    for (size_t i = 0; i < qd.size() && i < max_velocities_.size(); ++i) {
        if (std::abs(qd[i]) > max_velocities_[i]) {
            qd[i] = (qd[i] > 0 ? 1 : -1) * max_velocities_[i];
        }
    }
}

void RealtimeIKSolver::set_max_velocities(const std::vector<double>& max_velocities) {
    max_velocities_ = max_velocities;
    RCLCPP_INFO(logger_, "更新关节最大速度限制");
}

double RealtimeIKSolver::check_singularity(const KDL::JntArray& current_q) {
    if (jacobian_solver_->JntToJac(current_q, jacobian_) != KDL::SolverI::E_NOERROR) {
        return 1.0;  // 假设失败时是奇异点
    }

    // 转换为 Eigen 矩阵
    Eigen::MatrixXd J(6, chain_.getNrOfJoints());
    for (size_t i = 0; i < 6; ++i) {
        for (size_t j = 0; j < chain_.getNrOfJoints(); ++j) {
            J(i, j) = jacobian_(i, j);
        }
    }

    // 计算行列式的绝对值作为奇异度指数
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    double min_singular = svd.singularValues()(chain_.getNrOfJoints() - 1);
    double max_singular = svd.singularValues()(0);
    
    // 奇异度指数：越接近 0 越接近奇异点
    double singularity_index = std::abs(min_singular) / (std::abs(max_singular) + 1e-6);
    
    return 1.0 - std::tanh(singularity_index);  // 映射到 [0, 1]
}

}  // namespace mixcontroller
