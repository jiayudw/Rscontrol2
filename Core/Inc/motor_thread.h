#ifndef __MOTOR_THREAD_H
#define __MOTOR_THREAD_H

#include "main.h"
#include "motor_shared.h"

#define MOTOR_THREAD_HZ 100U
#define MOTOR_THREAD_PERIOD_MS (1000U / MOTOR_THREAD_HZ)

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
