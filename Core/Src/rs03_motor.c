#include "rs03_motor.h"
#include "can.h"
// 定义电机状态全局变量
RS03_MotorState_t motor_state;
// 将浮点数映射为无符号整数，用于 CAN 数据封包
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
// 【新增】灵足私有协议 - 生成 29 位扩展帧 ID
uint32_t RS03_GetExtId(uint8_t mode, uint8_t motor_id) {
    uint16_t master_id = 0xFD; // 根据你的Excel表，CAN_MASTER为253(0xFD)
    // 扩展帧格式: [28:24]为模式，[23:16]为主控ID，[15:8]为电机ID，[7:0]保留为0
    return ((uint32_t)mode << 24) | ((uint32_t)master_id << 8) | motor_id;
}
// 【新增】私有协议 - 使能电机 (Mode = 3)
void RS03_Private_Enable(CAN_HandleTypeDef *hcan, uint8_t id)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8] = {1, 0, 0, 0, 0, 0, 0, 0}; // 协议规定，使能时 data[0]=1

    tx_header.ExtId = RS03_GetExtId(3, id); // Mode 3
    tx_header.IDE = CAN_ID_EXT;             // 重点！改为扩展帧
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

// 【新增】私有协议 - 运控模式控制 (Mode = 1)
void RS03_Private_Control(CAN_HandleTypeDef *hcan, uint8_t id, float torque, float position, float speed, float kp)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    tx_header.ExtId = RS03_GetExtId(1, id); // Mode 1
    tx_header.IDE = CAN_ID_EXT;             // 重点！改为扩展帧
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    // 私有协议下，各项参数均映射为 16 位 (2个字节)
    int t_int = float_to_uint(torque, T_MIN, T_MAX, 16);
    int p_int = float_to_uint(position, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(speed, V_MIN, V_MAX, 16);
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 16);

    tx_data[0] = t_int >> 8;
    tx_data[1] = t_int & 0xFF;
    tx_data[2] = p_int >> 8;
    tx_data[3] = p_int & 0xFF;
    tx_data[4] = v_int >> 8;
    tx_data[5] = v_int & 0xFF;
    tx_data[6] = kp_int >> 8;
    tx_data[7] = kp_int & 0xFF;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}
// 将无符号整数映射回浮点数，用于解析反馈
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
// 运控模式发送数据帧
void RS03_MIT_Control(CAN_HandleTypeDef *hcan, uint8_t id, float torque, float position, float speed, float kp, float kd)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    // 配置 CAN 发送头 (MIT 协议使用的是标准数据帧)
    tx_header.StdId = id;              // 电机的 CAN ID
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;        // 标准帧
    tx_header.RTR = CAN_RTR_DATA;      // 数据帧
    tx_header.DLC = 8;                 // 8字节

    // 限制数据范围并转换为整数
    int p_int = float_to_uint(position, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(speed, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    int t_int = float_to_uint(torque, T_MIN, T_MAX, 12);

    // 根据 RS03 的协议格式进行封包
    tx_data[0] = p_int >> 8;                           // 角度高8位
    tx_data[1] = p_int & 0xFF;                         // 角度低8位
    tx_data[2] = v_int >> 4;                           // 速度高8位
    tx_data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // 速度低4位 + KP高4位
    tx_data[4] = kp_int & 0xFF;                        // KP低8位
    tx_data[5] = kd_int >> 4;                          // KD高8位
    tx_data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KD低4位 + 扭矩高4位
    tx_data[7] = t_int & 0xFF;                         // 扭矩低8位

    // 将报文放入发送邮箱
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

// 开启电机 (MIT协议，发送全0xFF，仅最后一字节为0xFC)
void RS03_MIT_Enable(CAN_HandleTypeDef *hcan, uint8_t id)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    tx_header.StdId = id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

// 关闭电机 (MIT协议，发送全0xFF，仅最后一字节为0xFD)
void RS03_MIT_Disable(CAN_HandleTypeDef *hcan, uint8_t id)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    tx_header.StdId = id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

// 解析电机的反馈报文 (MIT协议反馈帧)
void RS03_MIT_ParseFeedback(uint8_t *rx_data)
{
    // MIT协议反馈帧格式:
    // data[0] = id
    // data[1][2] = position (16位)
    // data[3][4] = speed (12位，占用[3]全和[4]高4位)
    // data[4][5] = torque (12位，占用[4]低4位和[5]全)
    // data[6][7] = temperature

    int p_int = (rx_data[1] << 8) | rx_data[2];
    int v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    int t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];

    motor_state.id = rx_data[0];
    motor_state.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    motor_state.speed = uint_to_float(v_int, V_MIN, V_MAX, 12);
    motor_state.torque = uint_to_float(t_int, T_MIN, T_MAX, 12);
    // 温度暂时不解析，如果需要可以按文档补充
}