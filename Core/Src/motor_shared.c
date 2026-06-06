#include "motor_shared.h"

volatile MotorCommand_t g_motor_commands[MOTOR_SLOT_COUNT];
volatile MotorState_t g_motor_states[MOTOR_SLOT_COUNT];
volatile float g_motor_zero_offsets[MOTOR_SLOT_COUNT];
volatile uint8_t g_motor_calibration_mode = 0U;

volatile float g_debug_target_position = 0.0f;
volatile float g_debug_target_speed = 0.0f;
volatile float g_debug_target_torque = 0.0f;
volatile float g_debug_kp = 0.5f;
volatile float g_debug_kd = 0.5f;

void MotorShared_Init(const MotorConfig_t *configs, uint8_t count)
{
    for (uint8_t i = 0; i < MOTOR_SLOT_COUNT; ++i) {
        g_motor_commands[i].target_position = 0.0f;
        g_motor_commands[i].target_speed = 0.0f;
        g_motor_commands[i].target_torque = 0.0f;
        g_motor_commands[i].kp = 0.5f;
        g_motor_commands[i].kd = 0.5f;
        g_motor_commands[i].enabled = 0U;
        g_motor_zero_offsets[i] = 0.0f;

        g_motor_states[i].index = i;
        g_motor_states[i].can_id = 0U;
        g_motor_states[i].motor_type = MOTOR_TYPE_RS;
        g_motor_states[i].motor_position = 0.0f;
        g_motor_states[i].motor_velocity = 0.0f;
        g_motor_states[i].motor_torque = 0.0f;
        g_motor_states[i].temperature = 0.0f;
        g_motor_states[i].joint_position = 0.0f;
        g_motor_states[i].joint_velocity = 0.0f;
        g_motor_states[i].mode_state = 0U;
        g_motor_states[i].fault_code = 0U;
        g_motor_states[i].is_enabled = 0U;
        g_motor_states[i].is_valid = 0U;
        g_motor_states[i].last_update_ms = 0U;
    }

    if (configs == 0) {
        return;
    }

    if (count > MOTOR_SLOT_COUNT) {
        count = MOTOR_SLOT_COUNT;
    }

    for (uint8_t i = 0; i < count; ++i) {
        uint8_t index = configs[i].index;
        if (index >= MOTOR_SLOT_COUNT) {
            continue;
        }

        g_motor_commands[index].enabled = configs[i].enabled;
        g_motor_zero_offsets[index] = configs[i].offset;
        g_motor_states[index].can_id = configs[i].can_id;
        g_motor_states[index].is_enabled = configs[i].enabled;
    }
}

int MotorShared_SetCommand(uint8_t index, float position, float speed, float kp, float kd, float torque)
{
    if (index >= MOTOR_SLOT_COUNT) {
        return 0;
    }

    g_motor_commands[index].target_position = position;
    g_motor_commands[index].target_speed = speed;
    g_motor_commands[index].kp = kp;
    g_motor_commands[index].kd = kd;
    g_motor_commands[index].target_torque = torque;

    if (index == 0U) {
        g_debug_target_position = position;
        g_debug_target_speed = speed;
        g_debug_kp = kp;
        g_debug_kd = kd;
        g_debug_target_torque = torque;
    }

    return 1;
}

void MotorShared_SetCalibrationMode(uint8_t enabled)
{
    g_motor_calibration_mode = enabled ? 1U : 0U;

    for (uint8_t i = 0; i < MOTOR_SLOT_COUNT; ++i) {
        g_motor_commands[i].target_position = 0.0f;
        g_motor_commands[i].target_speed = 0.0f;
        g_motor_commands[i].target_torque = 0.0f;
    }
}
