/**
 * PresetGraspDirection.h
 * 
 * 方法一：预设方向抓取位姿生成器
 * 
 * 设计原则：
 * 1. 提供语义化的预设方向接口
 * 2. 一行代码生成标准抓取位姿
 * 3. 支持最常用的抓取场景
 * 4. 代码可读性优先
 */

#ifndef PRESET_GRASP_DIRECTION_H
#define PRESET_GRASP_DIRECTION_H

#include <cmath>
#include <vector>
#include <iostream>
#include <string>
#include <functional>
#include <array>

namespace PresetGrasp {

// ========== 基础类型定义 ==========

// 角度常量
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;

// 末端姿态结构体
struct EndPose {
    double x, y, z;           // 位置 (米)
    double roll, pitch, yaw;  // 姿态 (弧度)
    
    EndPose() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
    
    EndPose(double x_, double y_, double z_, 
            double roll_, double pitch_, double yaw_)
        : x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_) {}
    
    // 获取位置向量
    std::vector<double> position() const { return {x, y, z}; }
    
    // 获取姿态向量（弧度）
    std::vector<double> orientation() const { return {roll, pitch, yaw}; }
    
    // 获取姿态向量（角度）
    std::vector<double> orientationDeg() const { 
        return {roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG}; 
    }
    
    // 打印位姿信息
    void print(const std::string& title = "EndPose") const {
        std::cout << "[" << title << "]" << std::endl;
        std::cout << "  位置: (" << x << ", " << y << ", " << z << ") m" << std::endl;
        std::cout << "  姿态: R=" << roll * RAD2DEG << "°, P=" 
                  << pitch * RAD2DEG << "°, Y=" << yaw * RAD2DEG << "°" << std::endl;
    }
    
    // 转换为字符串
    std::string toString() const {
        char buf[256];
        snprintf(buf, sizeof(buf), 
            "Pos(%.3f, %.3f, %.3f) Orient(R=%.1f°, P=%.1f°, Y=%.1f°)",
            x, y, z, roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
        return std::string(buf);
    }
};

// ========== 预设方向枚举 ==========

/**
 * 预设抓取方向
 * 
 * 坐标系约定：
 * - 机械臂基座位于原点 (0, 0, 0)
 * - X轴正方向：远离机械臂方向（前方）
 * - X轴负方向：朝向机械臂方向（后方）
 * - Y轴正方向：左侧
 * - Y轴负方向：右侧
 * - Z轴正方向：向上
 * - Z轴负方向：向下
 * 
 * 姿态角约定（ZYX顺序）：
 * - Roll (绕X轴)：控制手指左右倾斜
 * - Pitch (绕Y轴)：控制末端俯仰（前后倾斜）
 * - Yaw (绕Z轴)：控制末端朝向（左右旋转）
 */
enum class Direction {
    AWAY_FROM_ROBOT,   // 平行地面，远离机械臂（X轴正方向）
    TOWARD_ROBOT,      // 平行地面，朝向机械臂（X轴负方向）
    LEFT,              // 平行地面，左侧（Y轴正方向）
    RIGHT,             // 平行地面，右侧（Y轴负方向）
    UP,                // 末端垂直向上
    DOWN               // 末端垂直向下
};

// 方向名称映射
inline const char* directionToString(Direction dir) {
    switch (dir) {
        case Direction::AWAY_FROM_ROBOT: return "远离机械臂";
        case Direction::TOWARD_ROBOT:    return "朝向机械臂";
        case Direction::LEFT:            return "左侧";
        case Direction::RIGHT:           return "右侧";
        case Direction::UP:              return "向上";
        case Direction::DOWN:            return "向下";
        default: return "未知";
    }
}

// ========== 预设方向生成器类 ==========

/**
 * PresetDirectionGenerator
 * 
 * 预设方向抓取位姿生成器
 * 
 * 使用方法：
 * 1. 直接调用静态方法：PresetDirectionGenerator::Away(x, y, z)
 * 2. 创建实例后调用实例方法
 * 3. 使用便捷函数
 * 
 * 特点：
 * - 预设标准姿态：roll=0°, pitch=0°（平行地面）
 * - yaw角根据方向自动设置
 * - 位置由参数指定
 */
class PresetDirectionGenerator {
public:
    // ========== 核心静态方法（最常用） ==========
    
    /**
     * 生成平行地面远离机械臂的抓取位姿
     * 
     * 姿态设置：
     * - roll = 0°（手指水平，不左右倾斜）
     * - pitch = 0°（平行于地面）
     * - yaw = 0°（朝向X轴正方向，即远离机械臂）
     * 
     * 适用场景：
     * - 工件在机械臂前方
     * - 需要将工件向远离机械臂方向拖动后释放
     * - 典型的水平抓取场景
     * 
     * @param x X坐标（米），正方向远离机械臂
     * @param y Y坐标（米），正方向为左侧
     * @param z Z坐标（米），正方向向上
     * @return 抓取位姿
     */
    static EndPose Away(double x, double y, double z) {
        return EndPose(x, y, z, 0, 0, 0);
    }
    
    /**
     * 生成平行地面朝向机械臂的抓取位姿
     * 
     * 姿态设置：
     * - roll = 0°（手指水平）
     * - pitch = 0°（平行于地面）
     * - yaw = 180°（朝向X轴负方向，即朝向机械臂）
     * 
     * 适用场景：
     * - 工件在机械臂后方
     * - 需要将工件向机械臂方向拖动后释放
     * 
     * @param x X坐标（米）
     * @param y Y坐标（米）
     * @param z Z坐标（米）
     * @return 抓取位姿
     */
    static EndPose Toward(double x, double y, double z) {
        return EndPose(x, y, z, 0, 0, PI);
    }
    
    /**
     * 生成平行地面左侧的抓取位姿
     * 
     * 姿态设置：
     * - roll = 0°（手指水平）
     * - pitch = 0°（平行于地面）
     * - yaw = 90°（朝向Y轴正方向，即左侧）
     * 
     * 适用场景：
     * - 工件在机械臂左侧
     * - 需要向左侧拖动工件
     * 
     * @param x X坐标（米）
     * @param y Y坐标（米）
     * @param z Z坐标（米）
     * @return 抓取位姿
     */
    static EndPose Left(double x, double y, double z) {
        return EndPose(x, y, z, 0, 0, PI / 2);
    }
    
    /**
     * 生成平行地面右侧的抓取位姿
     * 
     * 姿态设置：
     * - roll = 0°（手指水平）
     * - pitch = 0°（平行于地面）
     * - yaw = -90°（朝向Y轴负方向，即右侧）
     * 
     * 适用场景：
     * - 工件在机械臂右侧
     * - 需要向右侧拖动工件
     * 
     * @param x X坐标（米）
     * @param y Y坐标（米）
     * @param z Z坐标（米）
     * @return 抓取位姿
     */
    static EndPose Right(double x, double y, double z) {
        return EndPose(x, y, z, 0, 0, -PI / 2);
    }
    
    /**
     * 生成末端垂直向上的抓取位姿
     * 
     * 姿态设置：
     * - roll = 0°
     * - pitch = 90°（末端指向+Z方向）
     * - yaw = 0°
     * 
     * @param x X坐标（米）
     * @param y Y坐标（米）
     * @param z Z坐标（米）
     * @return 抓取位姿
     */
    static EndPose Up(double x, double y, double z) {
        return EndPose(x, y, z, 0, PI / 2, 0);
    }
    
    /**
     * 生成末端垂直向下的抓取位姿
     * 
     * 姿态设置：
     * - roll = 0°
     * - pitch = -90°（末端指向-Z方向）
     * - yaw = 0°
     * 
     * 适用场景：
     * - 从上方抓取工件
     * - 典型的垂直抓取场景
     * 
     * @param x X坐标（米）
     * @param y Y坐标（米）
     * @param z Z坐标（米）
     * @return 抓取位姿
     */
    static EndPose Down(double x, double y, double z) {
        return EndPose(x, y, z, 0, -PI / 2, 0);
    }
    
    // ========== 使用枚举的通用方法 ==========
    
    /**
     * 根据预设方向生成抓取位姿
     * 
     * @param position 位置向量 [x, y, z]
     * @param dir 预设方向
     * @return 抓取位姿
     */
    static EndPose FromDirection(const std::vector<double>& position, 
                                  Direction dir) {
        double x = position.size() > 0 ? position[0] : 0;
        double y = position.size() > 1 ? position[1] : 0;
        double z = position.size() > 2 ? position[2] : 0;
        
        switch (dir) {
            case Direction::AWAY_FROM_ROBOT: return Away(x, y, z);
            case Direction::TOWARD_ROBOT:    return Toward(x, y, z);
            case Direction::LEFT:            return Left(x, y, z);
            case Direction::RIGHT:           return Right(x, y, z);
            case Direction::UP:              return Up(x, y, z);
            case Direction::DOWN:            return Down(x, y, z);
            default: return Away(x, y, z);
        }
    }
    
    // ========== 带变体的预设方向 ==========
    
    /**
     * 平行地面远离机械臂（带角度偏移）
     * 
     * 在基础方向上添加yaw角偏移，用于微调或特殊角度需求
     * 
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param yaw_offset_deg yaw角偏移（度），正值为逆时针
     * @return 抓取位姿
     */
    static EndPose AwayWithOffset(double x, double y, double z, 
                                   double yaw_offset_deg) {
        double yaw_offset_rad = yaw_offset_deg * DEG2RAD;
        return EndPose(x, y, z, 0, 0, yaw_offset_rad);
    }
    
    /**
     * 平行地面远离机械臂（带pitch偏移）
     * 
     * 用于需要轻微倾斜的抓取场景
     * 
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param pitch_deg pitch角（度），正值末端抬头，负值末端低头
     * @return 抓取位姿
     */
    static EndPose AwayWithPitch(double x, double y, double z, 
                                  double pitch_deg) {
        double pitch_rad = pitch_deg * DEG2RAD;
        return EndPose(x, y, z, 0, pitch_rad, 0);
    }
    
    /**
     * 远离机械臂方向（完全自定义）
     * 
     * 在基础方向上完全自定义姿态
     * 
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param roll_deg roll角（度）
     * @param pitch_deg pitch角（度）
     * @param yaw_deg yaw角（度）
     * @return 抓取位姿
     */
    static EndPose AwayCustom(double x, double y, double z,
                               double roll_deg, double pitch_deg, 
                               double yaw_deg) {
        return EndPose(x, y, z, 
                       roll_deg * DEG2RAD, 
                       pitch_deg * DEG2RAD, 
                       yaw_deg * DEG2RAD);
    }
};

// ========== 便捷函数命名空间 ==========

/**
 * QuickGrasp
 * 
 * 便捷函数命名空间
 * 提供最简单的一行代码接口
 * 
 * 使用方法：
 * EndPose pose = QuickGrasp::Away(0.3, 0.0, 0.15);
 */
namespace QuickGrasp {

// 便捷函数声明
inline EndPose Away(double x, double y, double z) {
    return PresetDirectionGenerator::Away(x, y, z);
}

inline EndPose Toward(double x, double y, double z) {
    return PresetDirectionGenerator::Toward(x, y, z);
}

inline EndPose Left(double x, double y, double z) {
    return PresetDirectionGenerator::Left(x, y, z);
}

inline EndPose Right(double x, double y, double z) {
    return PresetDirectionGenerator::Right(x, y, z);
}

inline EndPose Up(double x, double y, double z) {
    return PresetDirectionGenerator::Up(x, y, z);
}

inline EndPose Down(double x, double y, double z) {
    return PresetDirectionGenerator::Down(x, y, z);
}

// 使用位置向量
inline EndPose Away(const std::vector<double>& pos) {
    return PresetDirectionGenerator::Away(pos[0], pos[1], pos[2]);
}

inline EndPose Toward(const std::vector<double>& pos) {
    return PresetDirectionGenerator::Toward(pos[0], pos[1], pos[2]);
}

} // namespace QuickGrasp

// ========== 批量生成 ==========

/**
 * 批量生成多个方向的抓取位姿
 * 
 * @param position 抓取位置
 * @return 包含所有方向位姿的向量
 */
inline std::vector<EndPose> generateAllDirections(
    const std::vector<double>& position) {
    return {
        PresetDirectionGenerator::Away(position[0], position[1], position[2]),
        PresetDirectionGenerator::Toward(position[0], position[1], position[2]),
        PresetDirectionGenerator::Left(position[0], position[1], position[2]),
        PresetDirectionGenerator::Right(position[0], position[1], position[2]),
        PresetDirectionGenerator::Up(position[0], position[1], position[2]),
        PresetDirectionGenerator::Down(position[0], position[1], position[2])
    };
}

/**
 * 批量生成水平方向的抓取位姿
 * 
 * @param position 抓取位置
 * @return 四个水平方向（远离、朝向、左侧、右侧）的位姿向量
 */
inline std::vector<EndPose> generateHorizontalDirections(
    const std::vector<double>& position) {
    return {
        PresetDirectionGenerator::Away(position[0], position[1], position[2]),
        PresetDirectionGenerator::Toward(position[0], position[1], position[2]),
        PresetDirectionGenerator::Left(position[0], position[1], position[2]),
        PresetDirectionGenerator::Right(position[0], position[1], position[2])
    };
}

} // namespace PresetGrasp

#endif // PRESET_GRASP_DIRECTION_H
