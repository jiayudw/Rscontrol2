#include "motor_thread.h"

#include "rs_motor.h"

static CAN_HandleTypeDef *motor_hcan = 0;

static const float motor_startq_raw_rad[MOTOR_SLOT_COUNT] = {
    2.487f,
    4.439f,
    3.741f,
    2.155f,
    -7.015f,
    0.0f,
    0.0f,
};

static MotorConfig_t motor_configs[MOTOR_SLOT_COUNT] = {
    {0U, 0x7FU, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {1U, 0x01U, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {2U, 0x02U, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {3U, 0x03U, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {4U, 0x04U, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {5U, 0x05U, 1U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
    {6U, 0x06U, 0U, 1.0f, 0.0f, RS_MOTOR_P_MIN, RS_MOTOR_P_MAX},
};

static void MotorThread_ApplyStartqOffsets(void)
{
    for (uint8_t i = 0U; i < MOTOR_SLOT_COUNT; ++i) {
        motor_configs[i].offset = motor_startq_raw_rad[i];
    }
}

static int MotorThread_FindIndexByCanId(uint8_t can_id)
{
    for (uint8_t i = 0; i < MOTOR_SLOT_COUNT; ++i) {
        if (motor_configs[i].can_id == can_id) {
            return (int)i;
        }
    }

    return -1;
}

static HAL_StatusTypeDef MotorThread_SendFrame(CAN_TxHeaderTypeDef *header, uint8_t data[8])
{
    uint32_t mailbox = 0U;
    uint32_t start_ms = HAL_GetTick();

    while (HAL_CAN_GetTxMailboxesFreeLevel(motor_hcan) == 0U) {
        if ((HAL_GetTick() - start_ms) > 2U) {
            return HAL_TIMEOUT;
        }
    }

    return HAL_CAN_AddTxMessage(motor_hcan, header, data, &mailbox);
}

static void MotorThread_EnableAll(void)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];

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

void MotorThread_Init(CAN_HandleTypeDef *hcan)
{
    motor_hcan = hcan;
    MotorThread_ApplyStartqOffsets();
    MotorShared_Init(motor_configs, MOTOR_SLOT_COUNT);

    if (motor_hcan != 0) {
        MotorThread_EnableAll();
    }
}

uint8_t MotorThread_GetMotorCount(void)
{
    return MOTOR_SLOT_COUNT;
}

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
}
