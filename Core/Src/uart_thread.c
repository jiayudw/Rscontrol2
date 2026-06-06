#include "uart_thread.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motor_shared.h"
#include "usart.h"

#define UART_RX_BUFFER_SIZE 128
#define UART_LINE_BUFFER_SIZE 96

static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static uint8_t rx_byte = 0;

static char line_buffer[UART_LINE_BUFFER_SIZE];
static uint16_t line_len = 0;

static int32_t UartThread_FloatToMilli(float value)
{
    if (value >= 0.0f) {
        return (int32_t)((value * 1000.0f) + 0.5f);
    }

    return (int32_t)((value * 1000.0f) - 0.5f);
}

static int32_t UartThread_FloatToDeci(float value)
{
    if (value >= 0.0f) {
        return (int32_t)((value * 10.0f) + 0.5f);
    }

    return (int32_t)((value * 10.0f) - 0.5f);
}

static char *UartThread_SkipSpaces(char *text)
{
    while (*text == ' ' || *text == '\t') {
        ++text;
    }

    return text;
}

static int UartThread_ParseCmd(char *text, uint8_t *index, float *position,
                               float *speed, float *kp, float *kd, float *torque)
{
    char *cursor = UartThread_SkipSpaces(text);
    char *end = 0;
    long parsed_index = strtol(cursor, &end, 10);

    if (end != cursor && parsed_index >= 0 && parsed_index < MOTOR_SLOT_COUNT &&
        (*end == ' ' || *end == '\t')) {
        *index = (uint8_t)parsed_index;
        cursor = UartThread_SkipSpaces(end);
    } else {
        *index = 0U;
        cursor = UartThread_SkipSpaces(text);
    }

    *position = strtof(cursor, &end);
    if (end == cursor) {
        return 0;
    }
    cursor = UartThread_SkipSpaces(end);

    *speed = strtof(cursor, &end);
    if (end == cursor) {
        return 0;
    }
    cursor = UartThread_SkipSpaces(end);

    *kp = strtof(cursor, &end);
    if (end == cursor) {
        return 0;
    }
    cursor = UartThread_SkipSpaces(end);

    *kd = strtof(cursor, &end);
    if (end == cursor) {
        return 0;
    }
    cursor = UartThread_SkipSpaces(end);

    *torque = strtof(cursor, &end);
    return end != cursor;
}

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
    uint8_t index = 0U;
    float position = 0.0f;
    float speed = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque = 0.0f;

    if (strncmp(line, "cmd ", 4) == 0) {
        if (UartThread_ParseCmd(line + 4, &index, &position, &speed, &kp, &kd, &torque)) {
            MotorShared_SetCommand(index, position, speed, kp, kd, torque);
        }
    }
}

static void UartThread_SendMotorStates(void)
{
    char text[128];

    for (uint8_t i = 0; i < MOTOR_SLOT_COUNT; ++i) {
        volatile MotorState_t *state = &g_motor_states[i];
        int len = snprintf(
            text,
            sizeof(text),
            "state %u id %u en %u valid %u p_mrad %ld v_mrad_s %ld tau_mNm %ld temp_c10 %ld\r\n",
            (unsigned int)i,
            (unsigned int)state->can_id,
            (unsigned int)state->is_enabled,
            (unsigned int)state->is_valid,
            (long)UartThread_FloatToMilli(state->joint_position),
            (long)UartThread_FloatToMilli(state->joint_velocity),
            (long)UartThread_FloatToMilli(state->motor_torque),
            (long)UartThread_FloatToDeci(state->temperature)
        );

        if (len > 0) {
            if (len > (int)sizeof(text)) {
                len = (int)sizeof(text);
            }
            HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, 20);
        }
    }
}

void UartThread_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void UartThread_Run(void)
{
    uint8_t data = 0;

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

    UartThread_SendMotorStates();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        UartThread_PushByte(rx_byte);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
