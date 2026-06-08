#include "chassis.h"

#include "dji_motor.h"

#define CHASSIS_TIMEOUT_MS 100U
#define CHASSIS_MAX_VX 2.0f
#define CHASSIS_MAX_VY 2.0f
#define CHASSIS_MAX_WZ 6.0f
#define CHASSIS_WHEEL_RADIUS_M 0.08f
#define CHASSIS_ROTATE_RADIUS_M 0.37f
#define CHASSIS_GEAR_RATIO 19.0f
#define CHASSIS_RAD_S_TO_RPM 9.5492966f

typedef struct {
    float vx;
    float vy;
    float wz;
    uint32_t last_update_ms;
    uint8_t enabled;
} ChassisCommand_t;

static ChassisCommand_t chassis_command;

static float Chassis_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}

static int Chassis_IsFinite(float value)
{
    return value == value && value < 1000000.0f && value > -1000000.0f;
}

static float Chassis_WheelLinearToMotorRpm(float wheel_speed_m_s)
{
    float wheel_rad_s = wheel_speed_m_s / CHASSIS_WHEEL_RADIUS_M;
    return wheel_rad_s * CHASSIS_RAD_S_TO_RPM * CHASSIS_GEAR_RATIO;
}

void Chassis_Init(CAN_HandleTypeDef *hcan)
{
    chassis_command.vx = 0.0f;
    chassis_command.vy = 0.0f;
    chassis_command.wz = 0.0f;
    chassis_command.last_update_ms = 0U;
    chassis_command.enabled = 0U;
    DjiMotor_Init(hcan);
}

void Chassis_SetCommand(float vx, float vy, float wz)
{
    if (!Chassis_IsFinite(vx) || !Chassis_IsFinite(vy) || !Chassis_IsFinite(wz)) {
        return;
    }

    chassis_command.vx = Chassis_Clamp(vx, -CHASSIS_MAX_VX, CHASSIS_MAX_VX);
    chassis_command.vy = Chassis_Clamp(vy, -CHASSIS_MAX_VY, CHASSIS_MAX_VY);
    chassis_command.wz = Chassis_Clamp(wz, -CHASSIS_MAX_WZ, CHASSIS_MAX_WZ);
    chassis_command.last_update_ms = HAL_GetTick();
    chassis_command.enabled = 1U;
}

void Chassis_Stop(void)
{
    chassis_command.vx = 0.0f;
    chassis_command.vy = 0.0f;
    chassis_command.wz = 0.0f;
    chassis_command.enabled = 0U;
    DjiMotor_StopAll();
}

void Chassis_Run(void)
{
    uint32_t now = HAL_GetTick();
    float vx = chassis_command.vx;
    float vy = chassis_command.vy;
    float wz = chassis_command.wz;

    if (!chassis_command.enabled || ((now - chassis_command.last_update_ms) > CHASSIS_TIMEOUT_MS)) {
        vx = 0.0f;
        vy = 0.0f;
        wz = 0.0f;
    }

    float lf = vx - vy - wz * CHASSIS_ROTATE_RADIUS_M;
    float rf = vx + vy + wz * CHASSIS_ROTATE_RADIUS_M;
    float lb = vx + vy - wz * CHASSIS_ROTATE_RADIUS_M;
    float rb = vx - vy + wz * CHASSIS_ROTATE_RADIUS_M;

    DjiMotor_SetTargetRpm(0U, Chassis_WheelLinearToMotorRpm(lf));
    DjiMotor_SetTargetRpm(1U, Chassis_WheelLinearToMotorRpm(rf));
    DjiMotor_SetTargetRpm(2U, Chassis_WheelLinearToMotorRpm(lb));
    DjiMotor_SetTargetRpm(3U, Chassis_WheelLinearToMotorRpm(rb));
    DjiMotor_RunControl(1.0f / (float)CHASSIS_THREAD_HZ);
}
