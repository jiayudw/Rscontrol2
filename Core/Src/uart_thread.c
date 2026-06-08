#include "uart_thread.h"

#include <stdio.h>
#include <string.h>

#include "chassis.h"
#include "motor_shared.h"
#include "usart.h"

#define UART_RX_BUFFER_SIZE 128
#define UART_FRAME_HEADER 0xAAU
#define UART_CONTROL_HEADER 0xABU
#define UART_FRAME_TAIL 0x55U
#define UART_FRAME_PAYLOAD_SIZE 36U
#define UART_CONTROL_PAYLOAD_SIZE 1U
#define UART_POSITION_MOTOR_COUNT 6U
#define UART_CHASSIS_VX_INDEX 6U
#define UART_CHASSIS_VY_INDEX 7U
#define UART_CHASSIS_WZ_INDEX 8U
#define UART_COMMAND_SPEED 0.0f
#define UART_COMMAND_KP 1.1f
#define UART_COMMAND_KD 0.1f
#define UART_COMMAND_TORQUE 0.0f
#define UART_FRAME_TYPE_NONE 0U
#define UART_FRAME_TYPE_POSITION 1U
#define UART_FRAME_TYPE_CONTROL 2U
#define UART_CONTROL_NORMAL_MODE 0U
#define UART_CONTROL_CALIBRATION_MODE 1U
#define UART_TELEMETRY_PERIOD_MS 200U

static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static uint8_t rx_byte = 0;

static uint8_t frame_payload[UART_FRAME_PAYLOAD_SIZE];
static uint8_t frame_pos = 0U;
static uint8_t frame_type = UART_FRAME_TYPE_NONE;

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

static int UartThread_IsFiniteCommand(float value)
{
    return value == value && value < 1000000.0f && value > -1000000.0f;
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
        if (!UartThread_IsFiniteCommand(position)) {
            return;
        }
    }

    float chassis_vx = UartThread_ReadLeFloat(&payload[UART_CHASSIS_VX_INDEX * 4U]);
    float chassis_vy = UartThread_ReadLeFloat(&payload[UART_CHASSIS_VY_INDEX * 4U]);
    float chassis_wz = UartThread_ReadLeFloat(&payload[UART_CHASSIS_WZ_INDEX * 4U]);
    if (!UartThread_IsFiniteCommand(chassis_vx) ||
        !UartThread_IsFiniteCommand(chassis_vy) ||
        !UartThread_IsFiniteCommand(chassis_wz)) {
        return;
    }

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

    Chassis_SetCommand(chassis_vx, chassis_vy, chassis_wz);
}

static void UartThread_ParseControlFrame(uint8_t mode)
{
    if (mode == UART_CONTROL_CALIBRATION_MODE) {
        MotorShared_SetCalibrationMode(1U);
        return;
    }

    if (mode == UART_CONTROL_NORMAL_MODE) {
        MotorShared_SetCalibrationMode(0U);
    }
}

static void UartThread_ProcessByte(uint8_t data)
{
    if (frame_type == UART_FRAME_TYPE_NONE) {
        if (data == UART_FRAME_HEADER) {
            frame_type = UART_FRAME_TYPE_POSITION;
            frame_pos = 0U;
        } else if (data == UART_CONTROL_HEADER) {
            frame_type = UART_FRAME_TYPE_CONTROL;
            frame_pos = 0U;
        }
        return;
    }

    uint8_t payload_size = (frame_type == UART_FRAME_TYPE_POSITION) ?
        UART_FRAME_PAYLOAD_SIZE : UART_CONTROL_PAYLOAD_SIZE;

    if (frame_pos < payload_size) {
        frame_payload[frame_pos++] = data;
        return;
    }

    if (data == UART_FRAME_TAIL) {
        if (frame_type == UART_FRAME_TYPE_POSITION) {
            UartThread_ParseFrame(frame_payload);
        } else if (frame_type == UART_FRAME_TYPE_CONTROL) {
            UartThread_ParseControlFrame(frame_payload[0]);
        }
        frame_type = UART_FRAME_TYPE_NONE;
        frame_pos = 0U;
        return;
    }

    if (data == UART_FRAME_HEADER) {
        frame_type = UART_FRAME_TYPE_POSITION;
        frame_pos = 0U;
        return;
    }

    if (data == UART_CONTROL_HEADER) {
        frame_type = UART_FRAME_TYPE_CONTROL;
        frame_pos = 0U;
        return;
    }

    frame_type = UART_FRAME_TYPE_NONE;
    frame_pos = 0U;
}

static void UartThread_SendMotorStates(void)
{
    char text[192];

    for (uint8_t i = 0; i < MOTOR_SLOT_COUNT; ++i) {
        volatile MotorState_t *state = &g_motor_states[i];
        int len = snprintf(
            text,
            sizeof(text),
            "state %u id %u en %u valid %u mode %u p_mrad %ld raw_mrad %ld v_mrad_s %ld tau_mNm %ld temp_c10 %ld\r\n",
            (unsigned int)i,
            (unsigned int)state->can_id,
            (unsigned int)state->is_enabled,
            (unsigned int)state->is_valid,
            (unsigned int)g_motor_calibration_mode,
            (long)UartThread_FloatToMilli(state->joint_position),
            (long)UartThread_FloatToMilli(state->motor_position),
            (long)UartThread_FloatToMilli(state->joint_velocity),
            (long)UartThread_FloatToMilli(state->motor_torque),
            (long)UartThread_FloatToDeci(state->temperature)
        );

        if (len > 0) {
            if (len > (int)sizeof(text)) {
                len = (int)sizeof(text);
            }
            HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, 5);
        }
    }
}

void UartThread_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void UartThread_Run(void)
{
    static uint32_t last_telemetry_ms = 0U;
    uint8_t data = 0;

    while (UartThread_PopByte(&data)) {
        UartThread_ProcessByte(data);
    }

    uint32_t now = HAL_GetTick();
    if ((now - last_telemetry_ms) >= UART_TELEMETRY_PERIOD_MS) {
        UartThread_SendMotorStates();
        last_telemetry_ms = now;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        UartThread_PushByte(rx_byte);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
