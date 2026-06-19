#ifndef __FSI6_THREAD_H
#define __FSI6_THREAD_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Fsi6Thread_Init(void);
void Fsi6Thread_Run(void);
void Fsi6Thread_OnRxCplt(UART_HandleTypeDef *huart);
void Fsi6Thread_OnError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
