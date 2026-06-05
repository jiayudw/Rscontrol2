#include "rs05_motor.h"
#include "can.h"
#include <string.h>

// 定义电机状态全局变量
RS05_MotorState_t motor_state;
volatile MotorState_t motor_states[MOTOR_MAX_COUNT];

// RS05 运动控制参数范围 (依据说明书 4.4 节)
#define P_MIN  -12.57f
#define P_MAX   12.57f
#define V_MIN  -50.0f
#define V_MAX   50.0f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  5.0f
#define T_MIN  -5.5f
#define T_MAX   5.5f

// 浮点数转无符号整数
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// 无符号整数转浮点数
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// 【关键修正】灵足私有协议 - 生成 29 位扩展帧 ID
// 扩展帧格式: [28:24] 模式位, [23:8] 数据位(使能时为主站ID，运控时为力矩), [7:0] 电机ID
uint32_t RS05_GetExtId(uint8_t mode, uint16_t data, uint8_t motor_id) 
{
    return ((uint32_t)mode << 24) | ((uint32_t)data << 8) | motor_id;
}

void RS05_MotorState_InitDefaults(uint8_t id)
{
    if (id >= MOTOR_MAX_COUNT) {
        return;
    }

    motor_states[id].id = id;
    motor_states[id].motor_type = MOTOR_TYPE_RS05;
    motor_states[id].direction = 1.0f;
    motor_states[id].offset = 0.0f;
    motor_states[id].lower_limit = P_MIN;
    motor_states[id].upper_limit = P_MAX;
}

// 开启电机 (Mode = 3)
void RS05_Private_Enable(CAN_HandleTypeDef *hcan, uint8_t id)
{
    uint16_t master_id = 0xFD; // 主站 ID
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8] = {0};  // 数据域全为 0

    // Mode=3, 数据域传主站 ID, 目标为电机 ID
    tx_header.ExtId = RS05_GetExtId(3, master_id, id); 
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);

    if (id < MOTOR_MAX_COUNT) {
        RS05_MotorState_InitDefaults(id);
        motor_states[id].is_enabled = 1;
    }
}

// 关闭电机 / 停止运行 (Mode = 4)
void RS05_Private_Disable(CAN_HandleTypeDef *hcan, uint8_t id)
{
    uint16_t master_id = 0xFD;
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8] = {0}; 

    // Mode=4, 数据域传主站 ID
    tx_header.ExtId = RS05_GetExtId(4, master_id, id); 
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);

    if (id < MOTOR_MAX_COUNT) {
        motor_states[id].is_enabled = 0;
    }
}

// 运控模式控制指令 (Mode = 1)
void RS05_Private_Control(CAN_HandleTypeDef *hcan, uint8_t id, float torque, float position, float speed, float kp, float kd)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    // 私有协议下，所有参数均映射为 16 位
    int t_int = float_to_uint(torque, T_MIN, T_MAX, 16);
    int p_int = float_to_uint(position, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(speed, V_MIN, V_MAX, 16);
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 16);

    // Mode=1, 【力矩存放在 CAN ID 的 data 段中】
    tx_header.ExtId = RS05_GetExtId(1, (uint16_t)t_int, id); 
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    // 8 字节 Data 存放 Position、Speed、Kp、Kd
    tx_data[0] = p_int >> 8;
    tx_data[1] = p_int & 0xFF;
    tx_data[2] = v_int >> 8;
    tx_data[3] = v_int & 0xFF;
    tx_data[4] = kp_int >> 8;
    tx_data[5] = kp_int & 0xFF;
    tx_data[6] = kd_int >> 8;
    tx_data[7] = kd_int & 0xFF;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

// 解析电机的反馈报文 (Mode = 2)
void RS05_Private_ParseFeedback(uint8_t *rx_data, uint32_t ext_id)
{
    // 反馈帧 (Mode = 2)，格式: [28:24]=2, [15:8]=发送节点ID(电机ID), [7:0]=主站ID
    uint8_t id = (ext_id >> 8) & 0xFF;
    
    // 数据段:
    // data[0][1] = 位置反馈
    // data[2][3] = 速度反馈
    // data[4][5] = 力矩反馈
    // data[6][7] = 温度反馈

    int p_int = (rx_data[0] << 8) | rx_data[1];
    int v_int = (rx_data[2] << 8) | rx_data[3];
    int t_int = (rx_data[4] << 8) | rx_data[5];
    int temp_int = (rx_data[6] << 8) | rx_data[7];

    float position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float velocity = uint_to_float(v_int, V_MIN, V_MAX, 16);
    float torque = uint_to_float(t_int, T_MIN, T_MAX, 16);
    float temperature = (float)temp_int / 10.0f;

    motor_state.id = id;
    motor_state.position = position;
    motor_state.speed = velocity;
    motor_state.torque = torque;
    motor_state.temperature = temperature;

    if (id < MOTOR_MAX_COUNT) {
        volatile MotorState_t *state = &motor_states[id];

        if (state->direction != 1.0f && state->direction != -1.0f) {
            RS05_MotorState_InitDefaults(id);
        }

        state->id = id;
        state->motor_position = position;
        state->motor_velocity = velocity;
        state->motor_torque = torque;
        state->temperature = temperature;
        state->joint_position = (position - state->offset) * state->direction;
        state->joint_velocity = velocity * state->direction;
        state->mode_state = (ext_id >> 22) & 0x03;
        state->fault_code = (ext_id >> 16) & 0x3F;
        state->is_valid = 1;
        state->last_update_ms = HAL_GetTick();
    }
}
