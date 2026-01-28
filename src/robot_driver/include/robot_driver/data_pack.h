#ifndef __DATAPACK_H__
#define __DATAPACK_H__

// 定义电机和机械臂的数据结构

// 设置结构体按1字节对齐
#pragma pack(1)

typedef struct{
    float rad;      // 电机的关节角度
    float omega;    // 电机的角速度
    float torque;   // 电机输出力矩
}Motor_t;

typedef struct{
    int pack_type; // 数据包类型
    Motor_t joints[6]; // 机械臂关节
}Arm_t;

// 将字节对齐设置恢复为默认值（通常是8字节）
#pragma pack()

#endif