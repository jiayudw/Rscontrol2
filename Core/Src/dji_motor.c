#include "dji_motor.h"

#include <string.h>

#define DJI_MOTOR_CURRENT_LIMIT 12000.0f
#define DJI_MOTOR_TARGET_RPM_LIMIT 6000.0f
#define DJI_MOTOR_ONLINE_TIMEOUT_MS 100U
#define DJI_MOTOR_ZERO_TARGET_RPM 1.0f

static CAN_HandleTypeDef *dji_hcan = 0;
static DjiMotor_t dji_motors[DJI_MOTOR_COUNT];

static float DjiMotor_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}

static int16_t DjiMotor_ClampInt16(float value)
{
    value = DjiMotor_Clamp(value, -DJI_MOTOR_CURRENT_LIMIT, DJI_MOTOR_CURRENT_LIMIT);
    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}

static int DjiMotor_FindByCanId(uint32_t std_id)
{
    uint8_t motor_id = (uint8_t)(std_id - 0x200U);

    for (uint8_t i = 0U; i < DJI_MOTOR_COUNT; ++i) {
        if (dji_motors[i].id == motor_id) {
            return (int)i;
        }
    }

    return -1;
}

static HAL_StatusTypeDef DjiMotor_SendCurrents(void)
{
    if (dji_hcan == 0) {
        return HAL_ERROR;
    }

    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = {0};
    uint32_t mailbox = 0U;
    uint32_t start_ms = HAL_GetTick();

    header.StdId = 0x200U;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;

    for (uint8_t i = 0U; i < DJI_MOTOR_COUNT; ++i) {
        uint8_t slot = (uint8_t)(dji_motors[i].id - 1U);
        if (slot >= 4U) {
            continue;
        }

        int16_t current = dji_motors[i].output_current;
        data[slot * 2U] = (uint8_t)(current >> 8);
        data[slot * 2U + 1U] = (uint8_t)(current & 0xFF);
    }

    while (HAL_CAN_GetTxMailboxesFreeLevel(dji_hcan) == 0U) {
        if ((HAL_GetTick() - start_ms) > 2U) {
            return HAL_TIMEOUT;
        }
    }

    return HAL_CAN_AddTxMessage(dji_hcan, &header, data, &mailbox);
}

void DjiMotor_Init(CAN_HandleTypeDef *hcan)
{
    static const uint8_t ids[DJI_MOTOR_COUNT] = {
        1U, /* LF */
        2U, /* RF */
        4U, /* LB */
        3U, /* RB */
    };
    static const int8_t directions[DJI_MOTOR_COUNT] = {
        1,  /* LF */
        -1, /* RF */
        1,  /* LB */
        -1, /* RB */
    };

    dji_hcan = hcan;
    memset(dji_motors, 0, sizeof(dji_motors));

    for (uint8_t i = 0U; i < DJI_MOTOR_COUNT; ++i) {
        dji_motors[i].id = ids[i];
        dji_motors[i].direction = directions[i];
        Pid_Init(&dji_motors[i].speed_pid, 4.5f, 0.2f, 0.0f, 3000.0f, DJI_MOTOR_CURRENT_LIMIT);
    }
}

void DjiMotor_SetTargetRpm(uint8_t index, float target_rpm)
{
    if (index >= DJI_MOTOR_COUNT) {
        return;
    }

    dji_motors[index].target_rpm = DjiMotor_Clamp(target_rpm, -DJI_MOTOR_TARGET_RPM_LIMIT, DJI_MOTOR_TARGET_RPM_LIMIT);
}

void DjiMotor_StopAll(void)
{
    for (uint8_t i = 0U; i < DJI_MOTOR_COUNT; ++i) {
        dji_motors[i].target_rpm = 0.0f;
        dji_motors[i].output_current = 0;
        Pid_Reset(&dji_motors[i].speed_pid);
    }

    DjiMotor_SendCurrents();
}

void DjiMotor_RunControl(float dt_s)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0U; i < DJI_MOTOR_COUNT; ++i) {
        DjiMotor_t *motor = &dji_motors[i];
        float target = motor->target_rpm * (float)motor->direction;
        float feedback = (float)motor->speed_rpm;

        if (!motor->online || ((now - motor->last_update_ms) > DJI_MOTOR_ONLINE_TIMEOUT_MS)) {
            motor->output_current = 0;
            Pid_Reset(&motor->speed_pid);
            continue;
        }

        if (target < DJI_MOTOR_ZERO_TARGET_RPM && target > -DJI_MOTOR_ZERO_TARGET_RPM) {
            motor->output_current = 0;
            Pid_Reset(&motor->speed_pid);
            continue;
        }

        motor->output_current = DjiMotor_ClampInt16(Pid_Update(&motor->speed_pid, target, feedback, dt_s));
    }

    DjiMotor_SendCurrents();
}

void DjiMotor_OnCanFeedback(uint32_t std_id, const uint8_t data[8])
{
    if (data == 0 || std_id < DJI_MOTOR_CAN_ID_MIN || std_id > DJI_MOTOR_CAN_ID_MAX) {
        return;
    }

    int index = DjiMotor_FindByCanId(std_id);
    if (index < 0) {
        return;
    }

    DjiMotor_t *motor = &dji_motors[index];
    uint8_t was_online = motor->online;
    motor->last_ecd = motor->ecd;
    motor->ecd = ((uint16_t)data[0] << 8) | data[1];
    motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temperature = data[6];

    if (was_online) {
        int32_t diff = (int32_t)motor->ecd - (int32_t)motor->last_ecd;
        if (diff > 4096) {
            motor->round_count--;
        } else if (diff < -4096) {
            motor->round_count++;
        }
    }

    motor->total_angle_deg = (float)motor->round_count * 360.0f + ((float)motor->ecd * 360.0f / 8192.0f);
    motor->online = 1U;
    motor->last_update_ms = HAL_GetTick();
}

const DjiMotor_t *DjiMotor_GetState(uint8_t index)
{
    if (index >= DJI_MOTOR_COUNT) {
        return 0;
    }

    return &dji_motors[index];
}
