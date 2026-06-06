#ifndef __MOTOR_THREAD_H
#define __MOTOR_THREAD_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t motor_id;
    float target_position;
    float target_speed;
    float target_torque;
    float kp;
    float kd;
} MotorThreadCommand_t;

extern volatile float g_debug_target_position;
extern volatile float g_debug_target_speed;
extern volatile float g_debug_target_torque;
extern volatile float g_debug_kp;
extern volatile float g_debug_kd;

void MotorThread_Init(CAN_HandleTypeDef *hcan, uint8_t motor_id);
void MotorThread_Run10ms(void);
void MotorThread_SetTarget(float position, float speed, float kp, float kd, float torque);

#ifdef __cplusplus
}
#endif

#endif
