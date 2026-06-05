#ifndef __RS05_MOTOR_H
#define __RS05_MOTOR_H

#include "main.h"

// 针对 RS05 的参数范围限制 (根据说明书 4.4 节宏定义)
#define P_MIN -12.57f       // 位置最小值 (rad)
#define P_MAX 12.57f        // 位置最大值 (rad)
#define V_MIN -50.0f        // 速度最小值 (rad/s)
#define V_MAX 50.0f         // 速度最大值 (rad/s)
#define KP_MIN 0.0f         // 刚度最小值
#define KP_MAX 500.0f       // 刚度最大值
#define KD_MIN 0.0f         // 阻尼最小值
#define KD_MAX 5.0f         // 阻尼最大值
#define T_MIN -5.5f         // 扭矩最小值 (N.m) 
#define T_MAX 5.5f          // 扭矩最大值 (N.m) 

#define MOTOR_MAX_COUNT 128
#define MOTOR_TYPE_RS05 2

// 定义一个结构体来存储电机的反馈状态
typedef struct {
    uint8_t id;
    float position;
    float speed;
    float torque;
    float temperature;
} RS05_MotorState_t;

typedef struct {
    uint8_t id;
    uint8_t motor_type;

    float motor_position;  // raw motor position
    float motor_velocity;
    float motor_torque;
    float temperature;

    float joint_position;  // calibrated joint position
    float joint_velocity;

    float direction;       // +1 or -1
    float offset;          // rad
    float lower_limit;     // rad
    float upper_limit;     // rad

    uint8_t mode_state;
    uint8_t fault_code;
    uint8_t is_enabled;
    uint8_t is_valid;

    uint32_t last_update_ms;
} MotorState_t;

// 声明外部的电机状态变量
extern RS05_MotorState_t motor_state;
extern volatile MotorState_t motor_states[MOTOR_MAX_COUNT];

// 浮点数转整数的辅助函数
int float_to_uint(float x, float x_min, float x_max, int bits);

// RS05 开启/关闭电机 (私有协议格式)
void RS05_Private_Enable(CAN_HandleTypeDef *hcan, uint8_t id);
void RS05_Private_Disable(CAN_HandleTypeDef *hcan, uint8_t id);

// RS05 电机控制接口 (使用私有协议 Mode=1)
// 参数：
// hcan: 对应的 CAN 句柄 (如 &hcan1)
// id: 发送给电机的 CAN ID
// torque: 目标扭矩
// position: 目标角度
// speed: 目标速度
// kp: 刚度
// kd: 阻尼
void RS05_Private_Control(CAN_HandleTypeDef *hcan, uint8_t id, float torque, float position, float speed, float kp, float kd);

// 声明接收解析函数 (私有协议反馈需要扩展帧 ID 来提取电机 ID)
void RS05_Private_ParseFeedback(uint8_t *rx_data, uint32_t ext_id);
void RS05_MotorState_InitDefaults(uint8_t id);

#endif
