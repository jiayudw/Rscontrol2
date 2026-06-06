#ifndef __UART_THREAD_H
#define __UART_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define UART_THREAD_HZ 50U
#define UART_THREAD_PERIOD_MS (1000U / UART_THREAD_HZ)

void UartThread_Init(void);
void UartThread_Run(void);

#ifdef __cplusplus
}
#endif

#endif
