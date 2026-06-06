#include "uart_thread.h"

#include <stdio.h>
#include <string.h>

#include "motor_shared.h"
#include "usart.h"

#define UART_RX_BUFFER_SIZE 128
#define UART_FRAME_HEADER 0xAAU
#define UART_FRAME_TAIL 0x55U
#define UART_FRAME_PAYLOAD_SIZE 24U
#define UART_POSITION_MOTOR_COUNT 6U
#define UART_COMMAND_SPEED 0.0f
#define UART_COMMAND_KP 1.1f
#define UART_COMMAND_KD 0.1f
#define UART_COMMAND_TORQUE 0.0f

static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static uint8_t rx_byte = 0;

static uint8_t frame_payload[UART_FRAME_PAYLOAD_SIZE];
static uint8_t frame_pos = 0U;
static uint8_t frame_active = 0U;

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

static uint32_t UartThread_ReadLeU32(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static float UartThread_ReadLeFloat(const uint8_t *data)
{
    uint32_t raw = UartThread_ReadLeU32(data);
    float value = 0.0f;
    memcpy(&value, &raw, sizeof(value));
    return value;
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

static void UartThread_ParseFrame(const uint8_t payload[UART_FRAME_PAYLOAD_SIZE])
{
    for (uint8_t i = 0U; i < UART_POSITION_MOTOR_COUNT; ++i) {
        float position = UartThread_ReadLeFloat(&payload[i * 4U]);
        MotorShared_SetCommand(
            i,
            position,
            UART_COMMAND_SPEED,
            UART_COMMAND_KP,
            UART_COMMAND_KD,
            UART_COMMAND_TORQUE
        );
    }
}

static void UartThread_ProcessByte(uint8_t data)
{
    if (!frame_active) {
        if (data == UART_FRAME_HEADER) {
            frame_active = 1U;
            frame_pos = 0U;
        }
        return;
    }

    if (frame_pos < UART_FRAME_PAYLOAD_SIZE) {
        frame_payload[frame_pos++] = data;
        return;
    }

    if (data == UART_FRAME_TAIL) {
        UartThread_ParseFrame(frame_payload);
        frame_active = 0U;
        frame_pos = 0U;
        return;
    }

    if (data == UART_FRAME_HEADER) {
        frame_pos = 0U;
        return;
    }

    frame_active = 0U;
    frame_pos = 0U;
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
        UartThread_ProcessByte(data);
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
