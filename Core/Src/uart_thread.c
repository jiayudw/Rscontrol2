#include "uart_thread.h"

#include <stdlib.h>
#include <string.h>

#include "motor_thread.h"
#include "usart.h"

#define UART_RX_BUFFER_SIZE 128
#define UART_LINE_BUFFER_SIZE 96

static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static uint8_t rx_byte = 0;

static char line_buffer[UART_LINE_BUFFER_SIZE];
static uint16_t line_len = 0;
static uint32_t last_hello_ms = 0;

static void UartThread_PushByte(uint8_t data)
{
    uint16_t next = (uint16_t)((rx_head + 1U) % UART_RX_BUFFER_SIZE);
    if (next != rx_tail) {
        rx_buffer[rx_head] = data;
        rx_head = next;
    }
}

static int UartThread_PopByte(uint8_t *data)
{
    if (rx_tail == rx_head) {
        return 0;
    }

    *data = rx_buffer[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % UART_RX_BUFFER_SIZE);
    return 1;
}

static void UartThread_ParseLine(char *line)
{
    char *arg = 0;

    if (strncmp(line, "pos ", 4) == 0) {
        g_debug_target_position = strtof(line + 4, 0);
        return;
    }

    if (strncmp(line, "kp ", 3) == 0) {
        g_debug_kp = strtof(line + 3, 0);
        return;
    }

    if (strncmp(line, "kd ", 3) == 0) {
        g_debug_kd = strtof(line + 3, 0);
        return;
    }

    if (strncmp(line, "speed ", 6) == 0) {
        g_debug_target_speed = strtof(line + 6, 0);
        return;
    }

    if (strncmp(line, "torque ", 7) == 0) {
        g_debug_target_torque = strtof(line + 7, 0);
        return;
    }

    if (strncmp(line, "cmd ", 4) == 0) {
        arg = line + 4;
        g_debug_target_position = strtof(arg, &arg);
        g_debug_target_speed = strtof(arg, &arg);
        g_debug_kp = strtof(arg, &arg);
        g_debug_kd = strtof(arg, &arg);
        g_debug_target_torque = strtof(arg, &arg);
    }
}

void UartThread_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    last_hello_ms = HAL_GetTick();
}

void UartThread_Run(void)
{
    static const uint8_t hello[] = "helloworld\r\n";
    uint8_t data = 0;
    uint32_t now = HAL_GetTick();

    if ((now - last_hello_ms) >= 1000U) {
        HAL_UART_Transmit(&huart1, (uint8_t *)hello, sizeof(hello) - 1U, 10);
        last_hello_ms = now;
    }

    while (UartThread_PopByte(&data)) {
        if (data == '\r') {
            continue;
        }

        if (data == '\n') {
            line_buffer[line_len] = '\0';
            UartThread_ParseLine(line_buffer);
            line_len = 0;
            continue;
        }

        if (line_len < (UART_LINE_BUFFER_SIZE - 1U)) {
            line_buffer[line_len++] = (char)data;
        } else {
            line_len = 0;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        UartThread_PushByte(rx_byte);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
