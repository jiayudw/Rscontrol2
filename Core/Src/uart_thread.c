#include "uart_thread.h"

#include <stdio.h>
#include <string.h>

#include "chassis.h"
#include "dji_motor.h"
#include "fsi6_thread.h"
#include "motor_shared.h"
#include "rs_lift_motor.h"
#include "usart.h"

#define UART_RX_BUFFER_SIZE 128
#define UART_POSITION_FRAME_HEADER 0xAAU
#define UART_CHASSIS_FRAME_HEADER 0xBBU
#define UART_CONTROL_HEADER 0xABU
#define UART_FRAME_TAIL 0x55U
#define UART_COMMAND_FRAME_PAYLOAD_SIZE 27U
#define UART_CONTROL_PAYLOAD_SIZE 1U
#define UART_POSITION_MOTOR_COUNT 6U
#define UART_POSITION_FRAME_RESERVED_OFFSET 24U
#define UART_GRIPPER_MOTOR_INDEX 6U
#define UART_GRIPPER_OPEN 1U
#define UART_GRIPPER_CLOSE 0U
#define UART_GRIPPER_CLOSE_POSITION 0.444f
#define UART_GRIPPER_OPEN_POSITION (UART_GRIPPER_CLOSE_POSITION + 1.1f)
#define UART_CHASSIS_SLOT6_OFFSET 0U
#define UART_CHASSIS_VX_OFFSET 4U
#define UART_CHASSIS_VY_OFFSET 8U
#define UART_CHASSIS_WZ_OFFSET 12U
#define UART_CHASSIS_LIFT_OFFSET 16U
#define UART_CHASSIS_RESERVED_OFFSET 17U
#define UART_SLOT6_INDEX 6U
#define UART_COMMAND_SPEED 0.0f
#define UART_COMMAND_TORQUE 0.0f
#define UART_FRAME_TYPE_NONE 0U
#define UART_FRAME_TYPE_POSITION 1U
#define UART_FRAME_TYPE_CHASSIS 2U
#define UART_FRAME_TYPE_CONTROL 3U
#define UART_CONTROL_NORMAL_MODE 0U
#define UART_CONTROL_CALIBRATION_MODE 1U
#define UART_CONTROL_TELEMETRY_OFF 2U
#define UART_CONTROL_TELEMETRY_ON 3U
#define UART_TELEMETRY_PERIOD_MS 200U
#define UART_RX_WATCHDOG_MS 2000U

/* UART 接收中断只收 1 字节，这里用环形缓冲把中断和主循环解耦。 */
static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static uint8_t rx_byte = 0;

volatile uint32_t g_uart_rx_irq_count = 0U;
volatile uint32_t g_uart_error_count = 0U;
volatile uint32_t g_uart_last_error = 0U;
volatile uint8_t g_uart_last_rx_byte = 0U;

/* 当前正在拼接的上位机协议帧。 */
static uint8_t frame_payload[UART_COMMAND_FRAME_PAYLOAD_SIZE];
static uint8_t frame_pos = 0U;
static uint8_t frame_type = UART_FRAME_TYPE_NONE;
static uint8_t telemetry_enabled = 0U;

/* 将浮点数转换成毫单位整数，供串口文本遥测输出使用。 */
static int32_t UartThread_FloatToMilli(float value)
{
    if (value >= 0.0f) {
        return (int32_t)((value * 1000.0f) + 0.5f);
    }

    return (int32_t)((value * 1000.0f) - 0.5f);
}

/* 将浮点数转换成十分之一单位整数，供温度等遥测量输出使用。 */
static int32_t UartThread_FloatToDeci(float value)
{
    if (value >= 0.0f) {
        return (int32_t)((value * 10.0f) + 0.5f);
    }

    return (int32_t)((value * 10.0f) - 0.5f);
}

/* 从小端字节数组中读取一个 32 位无符号整数。 */
static uint32_t UartThread_ReadLeU32(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/* 从小端字节数组中读取一个 float32 数值。 */
static float UartThread_ReadLeFloat(const uint8_t *data)
{
    /* 上位机按小端 float32 发送，STM32 端手动组 uint32 后再转成 float。 */
    uint32_t raw = UartThread_ReadLeU32(data);
    float value = 0.0f;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

/* 判断上位机下发的浮点命令是否是有效有限值。 */
static int UartThread_IsFiniteCommand(float value)
{
    return value == value && value < 1000000.0f && value > -1000000.0f;
}

/* 将中断收到的 1 个字节写入 UART 环形缓冲区。 */
static void UartThread_PushByte(uint8_t data)
{
    uint16_t next = (uint16_t)((rx_head + 1U) % UART_RX_BUFFER_SIZE);
    if (next != rx_tail) {
        rx_buffer[rx_head] = data;
        rx_head = next;
    }
}

/* 从 UART 环形缓冲区取出 1 个待解析字节。 */
static int UartThread_PopByte(uint8_t *data)
{
    if (rx_tail == rx_head) {
        return 0;
    }

    *data = rx_buffer[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % UART_RX_BUFFER_SIZE);
    return 1;
}

static void UartThread_SetPositionCommand(uint8_t index, float position)
{
    /* 上位机只给位置；速度、刚度、阻尼、力矩由固件统一给默认值。 */
    MotorShared_SetCommand(
        index,
        position,
        UART_COMMAND_SPEED,
        g_motor_command_kp[index],
        g_motor_command_kd[index],
        UART_COMMAND_TORQUE
    );
}

/* 解析 0xAA 数据帧，并更新 ID0-ID5 机械臂位置命令。 */
static void UartThread_ParsePositionFrame(const uint8_t payload[UART_COMMAND_FRAME_PAYLOAD_SIZE])
{
    /* 0xAA 数据帧：6 个关节位置，全部是 little-endian float32，后 3 字节保留。 */
    for (uint8_t i = 0U; i < UART_POSITION_MOTOR_COUNT; ++i) {
        float position = UartThread_ReadLeFloat(&payload[i * 4U]);
        if (!UartThread_IsFiniteCommand(position)) {
            return;
        }
    }

    for (uint8_t i = 0U; i < UART_POSITION_MOTOR_COUNT; ++i) {
        float position = UartThread_ReadLeFloat(&payload[i * 4U]);
        UartThread_SetPositionCommand(i, position);
    }

    /* payload[25] 控制夹爪：1 = 开，0 = 关 */
    if (payload[25] == UART_GRIPPER_OPEN) {
        MotorShared_SetCommand(
            UART_GRIPPER_MOTOR_INDEX,
            UART_GRIPPER_OPEN_POSITION,
            0.0f,
            g_motor_command_kp[UART_GRIPPER_MOTOR_INDEX],
            g_motor_command_kd[UART_GRIPPER_MOTOR_INDEX],
            0.0f
        );
    } else if (payload[25] == UART_GRIPPER_CLOSE) {
        MotorShared_SetCommand(
            UART_GRIPPER_MOTOR_INDEX,
            UART_GRIPPER_CLOSE_POSITION,
            0.0f,
            g_motor_command_kp[UART_GRIPPER_MOTOR_INDEX],
            g_motor_command_kd[UART_GRIPPER_MOTOR_INDEX],
            0.0f
        );
    }
}

/* 解析 0xBB 数据帧，并更新 ID6 机械臂位置和底盘速度命令。 */
static void UartThread_ParseChassisFrame(const uint8_t payload[UART_COMMAND_FRAME_PAYLOAD_SIZE])
{
    (void)payload[UART_CHASSIS_RESERVED_OFFSET];

    float slot6_position = UartThread_ReadLeFloat(&payload[UART_CHASSIS_SLOT6_OFFSET]);
    float chassis_vx = UartThread_ReadLeFloat(&payload[UART_CHASSIS_VX_OFFSET]);
    float chassis_vy = UartThread_ReadLeFloat(&payload[UART_CHASSIS_VY_OFFSET]);
    float chassis_wz = UartThread_ReadLeFloat(&payload[UART_CHASSIS_WZ_OFFSET]);
    int8_t lift_cmd = (int8_t)payload[UART_CHASSIS_LIFT_OFFSET];
    (void)lift_cmd;

    if (!UartThread_IsFiniteCommand(slot6_position) ||
        !UartThread_IsFiniteCommand(chassis_vx) ||
        !UartThread_IsFiniteCommand(chassis_vy) ||
        !UartThread_IsFiniteCommand(chassis_wz)) {
        return;
    }

    /* 末端电机已改为遥控器 CH6 控制，这里不再通过 UART 下发 */
    /* UartThread_SetPositionCommand(UART_SLOT6_INDEX, slot6_position); */
    /* Chassis translation/turning is now driven by FS-i6 IBUS on USART6. */
    /* Chassis_SetCommand(chassis_vy, chassis_vx, -chassis_wz); */
    /* Old lift control used byte 16 from the 0xBB UART frame:
     * RsLiftMotor_SetCommand(lift_cmd);
     * Lift is now driven by FS-i6 CH5 in fsi6_thread.c.
     */
}

/* 解析 0xAB 控制帧，并切换校准模式或普通模式。 */
static void UartThread_ParseControlFrame(uint8_t mode)
{
    /* 0xAB 控制帧只控制工作模式，不修改零点参数。 */
    if (mode == UART_CONTROL_CALIBRATION_MODE) {
        MotorShared_SetCalibrationMode(1U);
        return;
    }

    if (mode == UART_CONTROL_NORMAL_MODE) {
        MotorShared_SetCalibrationMode(0U);
        return;
    }

    if (mode == UART_CONTROL_TELEMETRY_OFF) {
        telemetry_enabled = 0U;
        return;
    }

    if (mode == UART_CONTROL_TELEMETRY_ON) {
        telemetry_enabled = 1U;
    }
}

static uint8_t UartThread_GetPayloadSize(uint8_t type)
{
    if (type == UART_FRAME_TYPE_CONTROL) {
        return UART_CONTROL_PAYLOAD_SIZE;
    }

    return UART_COMMAND_FRAME_PAYLOAD_SIZE;
}

/* 按字节推进 UART 协议状态机，拼出完整上位机帧。
  如果当前没在收帧：
    收到 0xAA -> 认为是机械臂位置命令帧
    收到 0xBB -> 认为是 ID6 位置/底盘命令帧
    收到 0xAB -> 认为是控制模式帧
    其他字节丢掉

  如果已经收到帧头：
    继续收 payload

  payload 收够后：
    等最后一个字节 0x55

  如果帧尾正确：
    0xAA 帧 -> UartThread_ParsePositionFrame()
    0xBB 帧 -> UartThread_ParseChassisFrame()
    0xAB 帧 -> UartThread_ParseControlFrame()
 */
static void UartThread_ProcessByte(uint8_t data)
{
    /* 简单状态机：先等帧头，再收固定长度 payload，最后检查帧尾。 */
    if (frame_type == UART_FRAME_TYPE_NONE) {
        if (data == UART_POSITION_FRAME_HEADER) {
            frame_type = UART_FRAME_TYPE_POSITION;
            frame_pos = 0U;
        } else if (data == UART_CHASSIS_FRAME_HEADER) {
            frame_type = UART_FRAME_TYPE_CHASSIS;
            frame_pos = 0U;
        } else if (data == UART_CONTROL_HEADER) {
            frame_type = UART_FRAME_TYPE_CONTROL;
            frame_pos = 0U;
        }
        return;
    }

    uint8_t payload_size = UartThread_GetPayloadSize(frame_type);

    if (frame_pos < payload_size) {
        frame_payload[frame_pos++] = data;
        return;
    }

    if (data == UART_FRAME_TAIL) {
        if (frame_type == UART_FRAME_TYPE_POSITION) {
            UartThread_ParsePositionFrame(frame_payload);
        } else if (frame_type == UART_FRAME_TYPE_CHASSIS) {
            UartThread_ParseChassisFrame(frame_payload);
        } else if (frame_type == UART_FRAME_TYPE_CONTROL) {
            UartThread_ParseControlFrame(frame_payload[0]);
        }
        frame_type = UART_FRAME_TYPE_NONE;
        frame_pos = 0U;
        return;
    }

    if (data == UART_POSITION_FRAME_HEADER) {
        frame_type = UART_FRAME_TYPE_POSITION;
        frame_pos = 0U;
        return;
    }

    if (data == UART_CHASSIS_FRAME_HEADER) {
        frame_type = UART_FRAME_TYPE_CHASSIS;
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

/* 通过 UART 定时发送所有电机状态，供上位机调试和校准查看。 */
static void UartThread_SendMotorStates(void)
{
    char text[192];

    /* 定时向上位机发 ASCII 状态行，方便串口终端和 Python 脚本直接查看。 */
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

    for (uint8_t i = 0; i < DJI_MOTOR_COUNT; ++i) {
        const DjiMotor_t *state = DjiMotor_GetState(i);
        if (state == 0) {
            continue;
        }

        int len = snprintf(
            text,
            sizeof(text),
            "dji %u id %u online %u target_rpm %ld speed_rpm %d current %d out %d temp %u\r\n",
            (unsigned int)i,
            (unsigned int)state->id,
            (unsigned int)state->online,
            (long)state->target_rpm,
            (int)state->speed_rpm,
            (int)state->current_raw,
            (int)state->output_current,
            (unsigned int)state->temperature
        );

        if (len > 0) {
            if (len > (int)sizeof(text)) {
                len = (int)sizeof(text);
            }
            HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, 5);
        }
    }

    const RsLiftMotor_t *lift = RsLiftMotor_GetState();
    if (lift != 0) {
        int len = snprintf(
            text,
            sizeof(text),
            "rs_lift id %u online %u target_mrad_s %ld speed_mrad_s %ld raw_mrad %ld tau_mNm %ld temp_c10 %ld\r\n",
            (unsigned int)lift->id,
            (unsigned int)lift->online,
            (long)UartThread_FloatToMilli(lift->target_speed_rad_s),
            (long)UartThread_FloatToMilli(lift->velocity_rad_s),
            (long)UartThread_FloatToMilli(lift->position_rad),
            (long)UartThread_FloatToMilli(lift->torque_nm),
            (long)UartThread_FloatToDeci(lift->temperature_c)
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
    /* 先中止任何残留的 RX 传输，并刷新 USART1 硬件缓冲区 */
    (void)HAL_UART_AbortReceive_IT(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);

    /* 启动单字节中断接收，失败则重试一次 */
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1) != HAL_OK) {
        (void)HAL_UART_AbortReceive_IT(&huart1);
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        HAL_Delay(1);
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* 处理 UART 接收缓冲区中的数据，并按周期发送电机状态遥测。 */
void UartThread_Run(void)
{
    static uint32_t last_rx_irq_count = 0U;
    static uint32_t last_rx_check_ms = 0U;
    static uint32_t last_telemetry_ms = 0U;
    uint8_t data = 0;

    /* RX 看门狗：如果 2 秒内没收到任何字节，强制重新初始化 USART1 接收 */
    uint32_t now = HAL_GetTick();
    if (last_rx_check_ms == 0U) {
        last_rx_check_ms = now;
        last_rx_irq_count = g_uart_rx_irq_count;
    } else if ((now - last_rx_check_ms) > UART_RX_WATCHDOG_MS) {
        if (g_uart_rx_irq_count == last_rx_irq_count) {
            (void)HAL_UART_AbortReceive_IT(&huart1);
            __HAL_UART_CLEAR_PEFLAG(&huart1);
            HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        }
        last_rx_check_ms = now;
        last_rx_irq_count = g_uart_rx_irq_count;
    }

    while (UartThread_PopByte(&data)) {
        UartThread_ProcessByte(data);
    }

    if (telemetry_enabled && (now - last_telemetry_ms) >= UART_TELEMETRY_PERIOD_MS) {
        UartThread_SendMotorStates();
        last_telemetry_ms = now;
    }
}

/* HAL 串口接收完成回调，将 USART1 收到的字节放入环形缓冲区。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* 收到一个字节后立即重新打开下一次中断接收。 */
        g_uart_rx_irq_count++;
        g_uart_last_rx_byte = rx_byte;
        UartThread_PushByte(rx_byte);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        return;
    }

    Fsi6Thread_OnRxCplt(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        g_uart_error_count++;
        g_uart_last_error = huart->ErrorCode;
        /* 先中止当前传输，确保 HAL 状态机完全复位后再重启 */
        (void)HAL_UART_AbortReceive_IT(&huart1);
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        return;
    }

    Fsi6Thread_OnError(huart);
}
