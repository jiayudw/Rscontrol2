#include "rs_lift_motor.h"

#include "rs_motor.h"

#define RS_LIFT_CAN_ID 0x08U
#define RS_LIFT_DIRECTION 1
#define RS_LIFT_SEND_TIMEOUT_MS 2U
#define RS_LIFT_ONLINE_TIMEOUT_MS 100U
#define RS_LIFT_COMMAND_TIMEOUT_MS 100U
#define RS_LIFT_TARGET_SPEED_RAD_S 6.2831853f
#define RS_LIFT_TARGET_SPEED_LIMIT_RAD_S 20.0f
#define RS_LIFT_ZERO_SPEED_RAD_S 0.01f
#define RS_LIFT_SPEED_KP 0.0f
#define RS_LIFT_SPEED_KD 0.7f
#define RS_LIFT_TARGET_TORQUE_NM 0.0f

static CAN_HandleTypeDef *rs_lift_hcan = 0;
static RsLiftMotor_t rs_lift_motor;
static uint32_t rs_lift_last_command_ms = 0U;

volatile uint32_t g_rs_lift_control_count = 0U;
volatile uint32_t g_rs_lift_feedback_count = 0U;
volatile uint32_t g_rs_lift_online = 0U;
volatile uint32_t g_rs_lift_last_tx_status = 0U;
volatile uint32_t g_rs_lift_last_tx_error = 0U;
volatile int8_t g_rs_lift_cmd = 0;
volatile uint32_t g_rs_lift_timeout_count = 0U;

static float RsLiftMotor_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}

static HAL_StatusTypeDef RsLiftMotor_SendFrame(CAN_TxHeaderTypeDef *header, uint8_t data[8])
{
    if (rs_lift_hcan == 0) {
        return HAL_ERROR;
    }

    uint32_t mailbox = 0U;
    uint32_t start_ms = HAL_GetTick();

    while (HAL_CAN_GetTxMailboxesFreeLevel(rs_lift_hcan) == 0U) {
        if ((HAL_GetTick() - start_ms) > RS_LIFT_SEND_TIMEOUT_MS) {
            return HAL_TIMEOUT;
        }
    }

    return HAL_CAN_AddTxMessage(rs_lift_hcan, header, data, &mailbox);
}

static void RsLiftMotor_Enable(void)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];

    if (!rs_lift_motor.enabled) {
        return;
    }

    /* 抬升电机也使用 RS 协议，上电后先发使能帧，再允许速度命令。 */
    for (uint8_t retry = 0U; retry < 5U; ++retry) {
        RS_Motor_BuildEnableFrame(rs_lift_motor.id, &header, data);
        (void)RsLiftMotor_SendFrame(&header, data);
        HAL_Delay(10);
    }
}

void RsLiftMotor_Init(CAN_HandleTypeDef *hcan)
{
    rs_lift_hcan = hcan;

    /* 抬升独立挂在 CAN1，默认 ID 0x08，避开机械臂现有 0x01..0x07。 */
    rs_lift_motor.id = RS_LIFT_CAN_ID;
    rs_lift_motor.direction = RS_LIFT_DIRECTION;
    rs_lift_motor.enabled = 1U;
    rs_lift_motor.target_speed_rad_s = 0.0f;
    rs_lift_motor.target_position_rad = 0.0f;
    rs_lift_motor.position_rad = 0.0f;
    rs_lift_motor.velocity_rad_s = 0.0f;
    rs_lift_motor.torque_nm = 0.0f;
    rs_lift_motor.temperature_c = 0.0f;
    rs_lift_motor.mode_state = 0U;
    rs_lift_motor.fault_code = 0U;
    rs_lift_motor.online = 0U;
    rs_lift_motor.last_update_ms = 0U;
    rs_lift_last_command_ms = 0U;
    g_rs_lift_cmd = 0;

    if (rs_lift_hcan != 0) {
        RsLiftMotor_Enable();
    }
}

void RsLiftMotor_SetCommand(int8_t lift_cmd)
{
    if (lift_cmd > 0) {
        g_rs_lift_cmd = 1;
        rs_lift_motor.target_speed_rad_s = RS_LIFT_TARGET_SPEED_RAD_S;
        rs_lift_last_command_ms = HAL_GetTick();
        return;
    }

    if (lift_cmd < 0) {
        g_rs_lift_cmd = -1;
        rs_lift_motor.target_speed_rad_s = -RS_LIFT_TARGET_SPEED_RAD_S;
        rs_lift_last_command_ms = HAL_GetTick();
        return;
    }

    g_rs_lift_cmd = 0;
    rs_lift_motor.target_speed_rad_s = 0.0f;
    rs_lift_last_command_ms = HAL_GetTick();
}

void RsLiftMotor_Stop(void)
{
    g_rs_lift_cmd = 0;
    rs_lift_motor.target_speed_rad_s = 0.0f;
    RsLiftMotor_RunControl();
}

void RsLiftMotor_RunControl(void)
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t now = HAL_GetTick();

    if (!rs_lift_motor.enabled) {
        return;
    }

    g_rs_lift_control_count++;

    if (g_rs_lift_cmd != 0 &&
        ((now - rs_lift_last_command_ms) > RS_LIFT_COMMAND_TIMEOUT_MS)) {
        g_rs_lift_cmd = 0;
        rs_lift_motor.target_speed_rad_s = 0.0f;
        g_rs_lift_timeout_count++;
    }

    float target_speed = RsLiftMotor_Clamp(
        rs_lift_motor.target_speed_rad_s * (float)rs_lift_motor.direction,
        -RS_LIFT_TARGET_SPEED_LIMIT_RAD_S,
        RS_LIFT_TARGET_SPEED_LIMIT_RAD_S
    );
    if (target_speed < RS_LIFT_ZERO_SPEED_RAD_S &&
        target_speed > -RS_LIFT_ZERO_SPEED_RAD_S) {
        target_speed = 0.0f;
    }

    /* 抬升目前按速度控制处理：kp=0 关闭位置项，kd 提供速度跟随。
       后续如果加限位/回零/位置闭环，应在这个模块内部扩展。 */
    RS_Motor_BuildControlFrame(
        rs_lift_motor.id,
        RS_LIFT_TARGET_TORQUE_NM,
        rs_lift_motor.target_position_rad,
        target_speed,
        RS_LIFT_SPEED_KP,
        RS_LIFT_SPEED_KD,
        &header,
        data
    );
    g_rs_lift_last_tx_status = (uint32_t)RsLiftMotor_SendFrame(&header, data);
    g_rs_lift_last_tx_error = HAL_CAN_GetError(rs_lift_hcan);

    g_rs_lift_online = (
        rs_lift_motor.online &&
        ((now - rs_lift_motor.last_update_ms) <= RS_LIFT_ONLINE_TIMEOUT_MS)
    ) ? 1U : 0U;
}

void RsLiftMotor_OnCanFeedback(uint32_t ext_id, uint8_t data[8])
{
    RSMotorFeedback_t feedback;

    if (!RS_Motor_ParseFeedback(ext_id, data, &feedback)) {
        return;
    }

    if (feedback.can_id != rs_lift_motor.id) {
        return;
    }

    rs_lift_motor.position_rad = feedback.position;
    rs_lift_motor.velocity_rad_s = feedback.velocity * (float)rs_lift_motor.direction;
    rs_lift_motor.torque_nm = feedback.torque * (float)rs_lift_motor.direction;
    rs_lift_motor.temperature_c = feedback.temperature;
    rs_lift_motor.mode_state = feedback.mode_state;
    rs_lift_motor.fault_code = feedback.fault_code;
    rs_lift_motor.online = 1U;
    rs_lift_motor.last_update_ms = HAL_GetTick();
    rs_lift_motor.target_position_rad = feedback.position;
    g_rs_lift_feedback_count++;
}

const RsLiftMotor_t *RsLiftMotor_GetState(void)
{
    return &rs_lift_motor;
}
