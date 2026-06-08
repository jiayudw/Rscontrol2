#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_THREAD_HZ 200U
#define CHASSIS_THREAD_PERIOD_MS (1000U / CHASSIS_THREAD_HZ)

void Chassis_Init(CAN_HandleTypeDef *hcan);
void Chassis_SetCommand(float vx, float vy, float wz);
void Chassis_Run(void);
void Chassis_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
