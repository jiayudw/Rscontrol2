#include "fsi6_thread.h"

#include "chassis.h"
#include "motor_shared.h"
#include "rs_lift_motor.h"
#include "usart.h"

#define FSI6_IBUS_FRAME_SIZE 32U
#define FSI6_IBUS_HEADER0 0x20U
#define FSI6_IBUS_HEADER1 0x40U
#define FSI6_CHANNEL_COUNT 14U
#define FSI6_CH_RIGHT_X 0U
#define FSI6_CH_RIGHT_Y 1U
/* Old FS-i6 mapping only used CH1/CH2/CH4 for chassis:
 * CH1 right stick X, CH2 right stick Y, CH4 left stick X.
 */
/* #define FSI6_CH_LEFT_Y 2U */
#define FSI6_CH_LEFT_X 3U
/* Old tentative lift mapping used CH3 left stick Y:
 * #define FSI6_CH_LIFT 2U
 * Standard FS-i6 throttle often does not auto-center, so the active mapping
 * uses CH5 auxiliary three-position switch instead.
 */
#define FSI6_CH_LIFT 4U
#define FSI6_CH_GRIPPER 5U
#define FSI6_GRIPPER_MOTOR_INDEX 6U
#define FSI6_GRIPPER_OPEN_POSITION (FSI6_GRIPPER_CLOSE_POSITION+1.1f)
#define FSI6_GRIPPER_CLOSE_POSITION (4.3f)
#define FSI6_CHANNEL_MIN 1000.0f
#define FSI6_CHANNEL_CENTER 1500.0f
#define FSI6_CHANNEL_MAX 2000.0f
#define FSI6_CHANNEL_DEADBAND 0.06f
#define FSI6_LIFT_UP_THRESHOLD 0.35f
#define FSI6_LIFT_DOWN_THRESHOLD (-0.35f)
#define FSI6_MAX_VX 1.5f
#define FSI6_MAX_VY 1.5f
#define FSI6_MAX_WZ 3.0f
#define FSI6_TIMEOUT_MS 100U
#define FSI6_RX_WATCHDOG_MS 2000U

static uint8_t fsi6_rx_byte = 0U;
static uint8_t fsi6_frame[FSI6_IBUS_FRAME_SIZE];
static uint8_t fsi6_frame_pos = 0U;
static uint32_t fsi6_last_valid_ms = 0U;

volatile uint32_t g_fsi6_rx_irq_count = 0U;
volatile uint32_t g_fsi6_valid_frame_count = 0U;
volatile uint32_t g_fsi6_checksum_error_count = 0U;
volatile uint32_t g_fsi6_sync_error_count = 0U;
volatile uint32_t g_fsi6_error_count = 0U;
volatile uint32_t g_fsi6_receive_start_status = 0U;
volatile uint32_t g_fsi6_receive_restart_status = 0U;
volatile uint32_t g_fsi6_last_uart_error = 0U;
volatile uint32_t g_fsi6_last_uart_state = 0U;
volatile uint32_t g_fsi6_last_uart_rx_state = 0U;
volatile uint16_t g_fsi6_channels[FSI6_CHANNEL_COUNT];
volatile float g_fsi6_vx = 0.0f;
volatile float g_fsi6_vy = 0.0f;
volatile float g_fsi6_wz = 0.0f;
volatile int8_t g_fsi6_lift_cmd = 0;

static float Fsi6_NormalizeChannel(uint16_t value)
{
    float normalized = ((float)value - FSI6_CHANNEL_CENTER) /
                       ((FSI6_CHANNEL_MAX - FSI6_CHANNEL_MIN) * 0.5f);

    if (normalized > 1.0f) {
        normalized = 1.0f;
    } else if (normalized < -1.0f) {
        normalized = -1.0f;
    }

    if (normalized < FSI6_CHANNEL_DEADBAND && normalized > -FSI6_CHANNEL_DEADBAND) {
        normalized = 0.0f;
    }

    return normalized;
}

static void Fsi6_ApplyChannels(void)
{
    float right_x = Fsi6_NormalizeChannel(g_fsi6_channels[FSI6_CH_RIGHT_X]);
    float right_y = Fsi6_NormalizeChannel(g_fsi6_channels[FSI6_CH_RIGHT_Y]);
    float left_x = Fsi6_NormalizeChannel(g_fsi6_channels[FSI6_CH_LEFT_X]);
    float lift = Fsi6_NormalizeChannel(g_fsi6_channels[FSI6_CH_LIFT]);

    g_fsi6_vx = right_y * FSI6_MAX_VX;
    g_fsi6_vy = right_x * FSI6_MAX_VY;
    /* Left stick right turns the chassis clockwise; internal wz positive is CCW. */
    g_fsi6_wz = -left_x * FSI6_MAX_WZ;

    Chassis_SetCommand(g_fsi6_vx, g_fsi6_vy, g_fsi6_wz);

    if (lift < FSI6_LIFT_DOWN_THRESHOLD) {
        g_fsi6_lift_cmd = 1;
    } else if (lift > FSI6_LIFT_UP_THRESHOLD) {
        g_fsi6_lift_cmd = -1;
    } else {
        g_fsi6_lift_cmd = 0;
    }
    RsLiftMotor_SetCommand(g_fsi6_lift_cmd);
    /* 通道6：两档开关控制机械臂末端开合（开=0, 合=-1.4） */
    {
        uint16_t gripper_raw = g_fsi6_channels[FSI6_CH_GRIPPER];
        float gripper_target = (gripper_raw > 1500U)
            ? FSI6_GRIPPER_CLOSE_POSITION
            : FSI6_GRIPPER_OPEN_POSITION;
        MotorShared_SetCommand(
            FSI6_GRIPPER_MOTOR_INDEX,
            gripper_target,
            0.0f,
            g_motor_command_kp[FSI6_GRIPPER_MOTOR_INDEX],
            g_motor_command_kd[FSI6_GRIPPER_MOTOR_INDEX],
            0.0f
        );
    }

    fsi6_last_valid_ms = HAL_GetTick();
}

static void Fsi6_ProcessFrame(void)
{
    uint16_t checksum = 0xFFFFU;
    uint16_t received_checksum = ((uint16_t)fsi6_frame[31] << 8) | fsi6_frame[30];
    uint16_t channels[FSI6_CHANNEL_COUNT];

    if (fsi6_frame[0] != FSI6_IBUS_HEADER0 || fsi6_frame[1] != FSI6_IBUS_HEADER1) {
        g_fsi6_sync_error_count++;
        return;
    }

    for (uint8_t i = 0U; i < 30U; ++i) {
        checksum = (uint16_t)(checksum - fsi6_frame[i]);
    }

    if (checksum != received_checksum) {
        g_fsi6_checksum_error_count++;
        return;
    }

    for (uint8_t i = 0U; i < FSI6_CHANNEL_COUNT; ++i) {
        channels[i] = (uint16_t)fsi6_frame[(i * 2U) + 2U] |
                      ((uint16_t)fsi6_frame[(i * 2U) + 3U] << 8);
    }

    for (uint8_t i = 0U; i < FSI6_CHANNEL_COUNT; ++i) {
        g_fsi6_channels[i] = channels[i];
    }

    g_fsi6_valid_frame_count++;
    Fsi6_ApplyChannels();
}

static void Fsi6_PushByte(uint8_t data)
{
    if (fsi6_frame_pos == 0U && data != FSI6_IBUS_HEADER0) {
        g_fsi6_sync_error_count++;
        return;
    }

    if (fsi6_frame_pos == 1U && data != FSI6_IBUS_HEADER1) {
        fsi6_frame_pos = 0U;
        g_fsi6_sync_error_count++;
        if (data == FSI6_IBUS_HEADER0) {
            fsi6_frame[fsi6_frame_pos++] = data;
        }
        return;
    }

    fsi6_frame[fsi6_frame_pos++] = data;

    if (fsi6_frame_pos >= FSI6_IBUS_FRAME_SIZE) {
        fsi6_frame_pos = 0U;
        Fsi6_ProcessFrame();
    }
}

void Fsi6Thread_Init(void)
{
    /* 先中止任何残留的 RX 传输，并刷新 USART6 硬件缓冲区 */
    (void)HAL_UART_AbortReceive_IT(&huart6);
    __HAL_UART_CLEAR_PEFLAG(&huart6);

    /* 启动单字节中断接收，失败则重试一次 */
    if (HAL_UART_Receive_IT(&huart6, &fsi6_rx_byte, 1U) != HAL_OK) {
        (void)HAL_UART_AbortReceive_IT(&huart6);
        __HAL_UART_CLEAR_PEFLAG(&huart6);
        HAL_Delay(1);
    }
    g_fsi6_receive_start_status = (uint32_t)HAL_UART_Receive_IT(&huart6, &fsi6_rx_byte, 1U);
    g_fsi6_last_uart_state = (uint32_t)huart6.gState;
    g_fsi6_last_uart_rx_state = (uint32_t)huart6.RxState;
    g_fsi6_last_uart_error = huart6.ErrorCode;
}

void Fsi6Thread_Run(void)
{
    static uint32_t last_rx_irq_count = 0U;
    static uint32_t last_rx_check_ms = 0U;

    /* RX 看门狗：如果 2 秒内没收到任何字节，强制重新初始化 USART6 接收 */
    uint32_t now = HAL_GetTick();
    if (last_rx_check_ms == 0U) {
        last_rx_check_ms = now;
        last_rx_irq_count = g_fsi6_rx_irq_count;
    } else if ((now - last_rx_check_ms) > FSI6_RX_WATCHDOG_MS) {
        if (g_fsi6_rx_irq_count == last_rx_irq_count && g_fsi6_valid_frame_count == 0U) {
            (void)HAL_UART_AbortReceive_IT(&huart6);
            __HAL_UART_CLEAR_PEFLAG(&huart6);
            g_fsi6_receive_restart_status = (uint32_t)HAL_UART_Receive_IT(&huart6, &fsi6_rx_byte, 1U);
        }
        last_rx_check_ms = now;
        last_rx_irq_count = g_fsi6_rx_irq_count;
    }

    if (fsi6_last_valid_ms != 0U &&
        (now - fsi6_last_valid_ms) > FSI6_TIMEOUT_MS) {
        fsi6_last_valid_ms = 0U;
        g_fsi6_vx = 0.0f;
        g_fsi6_vy = 0.0f;
        g_fsi6_wz = 0.0f;
        g_fsi6_lift_cmd = 0;
        Chassis_SetCommand(0.0f, 0.0f, 0.0f);
        RsLiftMotor_SetCommand(0);
    }
}

void Fsi6Thread_OnRxCplt(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART6) {
        return;
    }

    g_fsi6_rx_irq_count++;
    Fsi6_PushByte(fsi6_rx_byte);
    g_fsi6_receive_restart_status = (uint32_t)HAL_UART_Receive_IT(&huart6, &fsi6_rx_byte, 1U);
    g_fsi6_last_uart_state = (uint32_t)huart6.gState;
    g_fsi6_last_uart_rx_state = (uint32_t)huart6.RxState;
    g_fsi6_last_uart_error = huart6.ErrorCode;
}

void Fsi6Thread_OnError(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART6) {
        return;
    }

    g_fsi6_error_count++;
    g_fsi6_last_uart_state = (uint32_t)huart6.gState;
    g_fsi6_last_uart_rx_state = (uint32_t)huart6.RxState;
    g_fsi6_last_uart_error = huart->ErrorCode;
    fsi6_frame_pos = 0U;

    /* 先中止当前传输，确保 HAL 状态机完全复位后再重启 */
    (void)HAL_UART_AbortReceive_IT(&huart6);
    __HAL_UART_CLEAR_PEFLAG(&huart6);
    g_fsi6_receive_restart_status = (uint32_t)HAL_UART_Receive_IT(&huart6, &fsi6_rx_byte, 1U);
}
