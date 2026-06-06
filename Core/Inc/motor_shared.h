#ifndef __MOTOR_SHARED_H
#define __MOTOR_SHARED_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_SLOT_COUNT 7U
#define MOTOR_TYPE_RS 2U

typedef struct {
    uint8_t index;
    uint8_t can_id;
    uint8_t enabled;
    float direction;
    float offset;
    float lower_limit;
    float upper_limit;
} MotorConfig_t;

typedef struct {
    float target_position;
    float target_speed;
    float target_torque;
    float kp;
    float kd;
    uint8_t enabled;
} MotorCommand_t;

typedef struct {
    uint8_t index;
    uint8_t can_id;
    uint8_t motor_type;

    float motor_position;
    float motor_velocity;
    float motor_torque;
    float temperature;

    float joint_position;
    float joint_velocity;

    uint8_t mode_state;
    uint8_t fault_code;
    uint8_t is_enabled;
    uint8_t is_valid;

    uint32_t last_update_ms;
} MotorState_t;

extern volatile MotorCommand_t g_motor_commands[MOTOR_SLOT_COUNT];
extern volatile MotorState_t g_motor_states[MOTOR_SLOT_COUNT];

extern volatile float g_debug_target_position;
extern volatile float g_debug_target_speed;
extern volatile float g_debug_target_torque;
extern volatile float g_debug_kp;
extern volatile float g_debug_kd;

void MotorShared_Init(const MotorConfig_t *configs, uint8_t count);
int MotorShared_SetCommand(uint8_t index, float position, float speed, float kp, float kd, float torque);

#ifdef __cplusplus
}
#endif

#endif
