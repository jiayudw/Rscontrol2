#ifndef __UART_THREAD_H
#define __UART_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void UartThread_Init(void);
void UartThread_Run(void);

#ifdef __cplusplus
}
#endif

#endif
