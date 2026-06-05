#ifndef __RS03_MOTOR_H
#define __RS03_MOTOR_H

#include "main.h"

// 针对 RS03 的参数范围限制 (根据说明书)
#define P_MIN -12.57f       // 位置最小值 (rad)
#define P_MAX 12.57f        // 位置最大值 (rad)
#define V_MIN -20.0f        // 速度最小值 (rad/s)
#define V_MAX 20.0f         // 速度最大值 (rad/s)
#define KP_MIN 0.0f         // 刚度最小值
#define KP_MAX 5000.0f      // 刚度最大值
#define KD_MIN 0.0f         // 阻尼最小值
#define KD_MAX 100.0f       // 阻尼最大值
#define T_MIN -60.0f        // 扭矩最小值 (N.m) - RS03是60Nm
#define T_MAX 60.0f         // 扭矩最大值 (N.m)

// 定义一个结构体来存储电机的反馈状态
typedef struct {
    uint8_t id;
    float position;
    float speed;
    float torque;
    float temperature;
} RS03_MotorState_t;


// 浮点数转整数的辅助函数
int float_to_uint(float x, float x_min, float x_max, int bits);

// RS03 电机控制接口 (使用 MIT 协议)
// 参数：
// hcan: 对应的 CAN 句柄 (如 &hcan1)
// id: 发送给电机的 CAN ID
// torque: 目标扭矩
// position: 目标角度
// speed: 目标速度
// kp: 刚度
// kd: 阻尼
void RS03_MIT_Control(CAN_HandleTypeDef *hcan, uint8_t id, float torque, float position, float speed, float kp, float kd);

// RS03 开启/关闭电机 (MIT协议格式)
void RS03_MIT_Enable(CAN_HandleTypeDef *hcan, uint8_t id);
void RS03_MIT_Disable(CAN_HandleTypeDef *hcan, uint8_t id);
// 声明外部的电机状态变量
extern RS03_MotorState_t motor_state;
// 声明接收解析函数
void RS03_MIT_ParseFeedback(uint8_t *rx_data);
// 声明私有协议函数
void RS03_Private_Enable(CAN_HandleTypeDef *hcan, uint8_t id);
void RS03_Private_Control(CAN_HandleTypeDef *hcan, uint8_t id, float torque, float position, float speed, float kp);

#endif