#ifndef __DATAPACK_H__
#define __DATAPACK_H__

#pragma pack(1)

typedef struct{
    float rad;
    float omega;
    float torque;
}Motor_t;

typedef struct{
    int pack_type;
    Motor_t joints[6];
}Arm_t;


#pragma pack()

#endif