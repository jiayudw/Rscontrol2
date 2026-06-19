#include "motor_thread.h"

#include "rs_motor.h"

static CAN_HandleTypeDef *motor_hcan = 0;

/* 每个关节的机械零位对应的电机原始角度，单位 rad。 */
static const float motor_startq_raw_rad[MOTOR_SLOT_COUNT] = {
    3.541f,
    4.447f,
    3.740f,
    2.152f,
    2.960f,
    2.830f,
    0.0f,
};

static MotorConfig_t motor_configs[MOTOR_SLOT_COUNT] = {
    /* index, CAN ID, enabled, direction, offset, lower_limit, upper_limit */
    {0U, 0x7FU, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {1U, 0x01U, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {2U, 0x02U, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {3U, 0x03U, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {4U, 0x04U, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {5U, 0x05U, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {6U, 0x06U, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
};

/* 将固定机械零位表写入运行时电机配置。 */
static void MotorThread_ApplyStartqOffsets(void)
{
    for (uint8_t i = 0U; i < MOTOR_SLOT_COUNT; ++i) {
        motor_configs[i].offset = motor_startq_raw_rad[i];
    }
}

/* 根据 RS 电机 CAN ID 查找对应的电机槽位。 */
static int MotorThread_FindIndexByCanId(uint8_t can_id)
{
    for (uint8_t i = 0; i < MOTOR_SLOT_COUNT; ++i) {
        if (motor_configs[i].can_id == can_id) {
            return (int)i;
        }
    }

    return -1;
}

/* 等待 CAN 发送邮箱可用，并发送一帧 RS 电机 CAN 报文。 */
static HAL_StatusTypeDef MotorThread_SendFrame(CAN_TxHeaderTypeDef *header, uint8_t data[8])
{
    uint32_t mailbox = 0U;
    uint32_t start_ms = HAL_GetTick();

    /* 等待发送邮箱，但最多等 2ms，避免主循环被 CAN 堵死。 */
    while (HAL_CAN_GetTxMailboxesFreeLevel(motor_hcan) == 0U) {
        if ((HAL_GetTick() - start_ms) > 2U) {
            return HAL_TIMEOUT;
        }
    }

    return HAL_CAN_AddTxMessage(motor_hcan, header, data, &mailbox);
}

/* 给所有启用的 RS 电机发送使能命令。 */
static void MotorThread_EnableAll(void)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];

    /* 上电后给每个启用的 RS 电机重复发使能帧，提高启动成功率。 */
    for (uint8_t i = 0; i < MOTOR_SLOT_COUNT; ++i) {
        if (!motor_configs[i].enabled) {
            continue;
        }

        for (uint8_t retry = 0; retry < 5U; ++retry) {
            RS_Motor_BuildEnableFrame(motor_configs[i].can_id, &header, data);
            MotorThread_SendFrame(&header, data);
            HAL_Delay(10);
        }
    }
}

/* 初始化 RS 电机线程、零点配置和共享状态。 */
void MotorThread_Init(CAN_HandleTypeDef *hcan)
{
    motor_hcan = hcan;
    MotorThread_ApplyStartqOffsets();
    MotorShared_Init(motor_configs, MOTOR_SLOT_COUNT);

    if (motor_hcan != 0) {
        MotorThread_EnableAll();
    }
}

/* 返回当前固件配置的 RS 电机槽位数量。 */
uint8_t MotorThread_GetMotorCount(void)
{
    return MOTOR_SLOT_COUNT;
}

/* 每 10ms 将共享命令转换为 RS 电机 CAN 控制帧并下发。 */
void MotorThread_Run10ms(void)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];

    if (motor_hcan == 0) {
        return;
    }

    for (uint8_t i = 0; i < MOTOR_SLOT_COUNT; ++i) {
        const MotorConfig_t *config = &motor_configs[i];
        volatile MotorCommand_t *command = &g_motor_commands[i];

        if (!config->enabled || !command->enabled) {
            continue;
        }

        /* 上层命令是关节坐标；下发 CAN 前转换成电机原始坐标。 */
        float offset = g_motor_zero_offsets[i];
        float target_position = g_motor_calibration_mode ? 0.0f : command->target_position;
        float motor_position = target_position * config->direction + offset;
        float motor_speed = g_motor_calibration_mode ? 0.0f : command->target_speed * config->direction;
        float motor_torque = g_motor_calibration_mode ? 0.0f : command->target_torque * config->direction;
        float kp = g_motor_calibration_mode ? 0.0f : command->kp;
        float kd = g_motor_calibration_mode ? 0.0f : command->kd;

        RS_Motor_BuildControlFrame(
            config->can_id,
            motor_torque,
            motor_position,
            motor_speed,
            kp,
            kd,
            &header,
            data
        );
        MotorThread_SendFrame(&header, data);
    }
}

/* 解析 RS 电机 CAN 反馈帧，并更新共享电机状态。 */
void MotorThread_OnCanFeedback(uint32_t ext_id, uint8_t data[8])
{
    RSMotorFeedback_t feedback;

    if (!RS_Motor_ParseFeedback(ext_id, data, &feedback)) {
        return;
    }

    int index = MotorThread_FindIndexByCanId(feedback.can_id);
    if (index < 0) {
        return;
    }

    const MotorConfig_t *config = &motor_configs[index];
    volatile MotorState_t *state = &g_motor_states[index];

    /* 反馈是电机原始坐标，减零点并乘方向后得到关节坐标。 */
    float joint_position = (feedback.position - g_motor_zero_offsets[index]) * config->direction;

    state->index = (uint8_t)index;
    state->can_id = feedback.can_id;
    state->motor_type = MOTOR_TYPE_RS;
    state->motor_position = feedback.position;
    state->motor_velocity = feedback.velocity;
    state->motor_torque = feedback.torque;
    state->temperature = feedback.temperature;
    state->joint_position = joint_position;
    state->joint_velocity = feedback.velocity * config->direction;
    state->mode_state = feedback.mode_state;
    state->fault_code = feedback.fault_code;
    state->is_enabled = config->enabled;
    state->is_valid = 1U;
    state->last_update_ms = HAL_GetTick();
    g_motor_position_error[index] = g_motor_commands[index].target_position - joint_position;
}
