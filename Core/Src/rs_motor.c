#include "rs_motor.h"

/* 将 RS 电机物理量按协议范围线性编码为整数。 */
int RS_Motor_FloatToUint(float x, float x_min, float x_max, int bits)
{
    /* 把物理量按协议范围线性映射到无符号整数编码。 */
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) {
        x = x_max;
    } else if (x < x_min) {
        x = x_min;
    }

    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/* 将 RS 电机协议整数按范围线性解码为物理量。 */
float RS_Motor_UintToFloat(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* 按 RS 电机协议拼接扩展帧 ID。 */
uint32_t RS_Motor_GetExtId(uint8_t mode, uint16_t data, uint8_t motor_id)
{
    /* RS 电机扩展帧 ID：高位放模式，中间放数据字段，低位放电机 ID。 */
    return ((uint32_t)mode << 24) | ((uint32_t)data << 8) | motor_id;
}

/* 构造 RS 电机使能 CAN 帧。 */
void RS_Motor_BuildEnableFrame(uint8_t id, CAN_TxHeaderTypeDef *header, uint8_t data[8])
{
    uint16_t master_id = 0xFD;

    header->ExtId = RS_Motor_GetExtId(3, master_id, id);
    header->IDE = CAN_ID_EXT;
    header->RTR = CAN_RTR_DATA;
    header->DLC = 8;

    for (uint8_t i = 0; i < 8U; ++i) {
        data[i] = 0U;
    }
}

/* 构造 RS 电机失能 CAN 帧。 */
void RS_Motor_BuildDisableFrame(uint8_t id, CAN_TxHeaderTypeDef *header, uint8_t data[8])
{
    uint16_t master_id = 0xFD;

    header->ExtId = RS_Motor_GetExtId(4, master_id, id);
    header->IDE = CAN_ID_EXT;
    header->RTR = CAN_RTR_DATA;
    header->DLC = 8;

    for (uint8_t i = 0; i < 8U; ++i) {
        data[i] = 0U;
    }
}

/* 构造 RS 电机 MIT 模式控制 CAN 帧。 */
void RS_Motor_BuildControlFrame(uint8_t id, float torque, float position, float speed,
                                float kp, float kd, CAN_TxHeaderTypeDef *header, uint8_t data[8])
{
    /* 控制帧：力矩放在扩展 ID 的 data 字段，其余控制量放 8 字节数据区。 */
    int t_int = RS_Motor_FloatToUint(torque, RS_MOTOR_T_MIN, RS_MOTOR_T_MAX, 16);
    int p_int = RS_Motor_FloatToUint(position, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX, 16);
    int v_int = RS_Motor_FloatToUint(speed, RS_MOTOR_V_MIN, RS_MOTOR_V_MAX, 16);
    int kp_int = RS_Motor_FloatToUint(kp, RS_MOTOR_KP_MIN, RS_MOTOR_KP_MAX, 16);
    int kd_int = RS_Motor_FloatToUint(kd, RS_MOTOR_KD_MIN, RS_MOTOR_KD_MAX, 16);

    header->ExtId = RS_Motor_GetExtId(1, (uint16_t)t_int, id);
    header->IDE = CAN_ID_EXT;
    header->RTR = CAN_RTR_DATA;
    header->DLC = 8;

    data[0] = (uint8_t)(p_int >> 8);
    data[1] = (uint8_t)(p_int & 0xFF);
    data[2] = (uint8_t)(v_int >> 8);
    data[3] = (uint8_t)(v_int & 0xFF);
    data[4] = (uint8_t)(kp_int >> 8);
    data[5] = (uint8_t)(kp_int & 0xFF);
    data[6] = (uint8_t)(kd_int >> 8);
    data[7] = (uint8_t)(kd_int & 0xFF);
}

/* 解析 RS 电机反馈 CAN 帧并转换为物理量。 */
int RS_Motor_ParseFeedback(uint32_t ext_id, const uint8_t data[8], RSMotorFeedback_t *feedback)
{
    uint8_t mode = (uint8_t)((ext_id >> 24) & 0x1F);
    /* 只处理 RS 电机反馈模式帧，其他模式直接丢弃。 */
    if (feedback == 0 || data == 0 || mode != 2U) {
        return 0;
    }

    int p_int = ((int)data[0] << 8) | data[1];
    int v_int = ((int)data[2] << 8) | data[3];
    int t_int = ((int)data[4] << 8) | data[5];
    int temp_int = ((int)data[6] << 8) | data[7];

    feedback->can_id = (uint8_t)((ext_id >> 8) & 0xFF);
    feedback->position = RS_Motor_UintToFloat(p_int, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX, 16);
    feedback->velocity = RS_Motor_UintToFloat(v_int, RS_MOTOR_V_MIN, RS_MOTOR_V_MAX, 16);
    feedback->torque = RS_Motor_UintToFloat(t_int, RS_MOTOR_T_MIN, RS_MOTOR_T_MAX, 16);
    feedback->temperature = (float)temp_int / 10.0f;
    feedback->mode_state = (uint8_t)((ext_id >> 22) & 0x03);
    feedback->fault_code = (uint8_t)((ext_id >> 16) & 0x3F);

    return 1;
}
