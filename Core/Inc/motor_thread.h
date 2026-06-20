#ifndef __MOTOR_THREAD_H
#define __MOTOR_THREAD_H

#include "main.h"
#include "motor_shared.h"

#define MOTOR_THREAD_HZ 100U
#define MOTOR_THREAD_PERIOD_MS (1000U / MOTOR_THREAD_HZ)

typedef enum {
    RS_MOTOR_STATE_INIT = 0,
    RS_MOTOR_STATE_ENABLING,
    RS_MOTOR_STATE_ACTIVE,
    RS_MOTOR_STATE_TIMEOUT,
} RSMotorState_t;

extern volatile uint8_t g_rs_motor_state[MOTOR_SLOT_COUNT];
extern volatile uint32_t g_rs_motor_enable_count;
extern volatile uint32_t g_rs_motor_timeout_count;
extern volatile uint32_t g_rs_motor_active_count;

#ifdef __cplusplus
extern "C" {
#endif

void MotorThread_Init(CAN_HandleTypeDef *hcan);
void MotorThread_Run10ms(void);
void MotorThread_OnCanFeedback(uint32_t ext_id, uint8_t data[8]);
uint8_t MotorThread_GetMotorCount(void);

#ifdef __cplusplus
}
#endif

#endif
