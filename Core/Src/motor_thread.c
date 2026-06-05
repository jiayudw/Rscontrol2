#include "motor_thread.h"

#include "rs05_motor.h"

volatile float g_debug_target_position = 0.0f;
volatile float g_debug_target_speed = 0.0f;
volatile float g_debug_target_torque = 0.0f;
volatile float g_debug_kp = 0.5f;
volatile float g_debug_kd = 0.5f;

static CAN_HandleTypeDef *motor_hcan = 0;
static MotorThreadCommand_t motor_cmd = {
    .motor_id = 0,
    .target_position = 0.0f,
    .target_speed = 0.0f,
    .target_torque = 0.0f,
    .kp = 0.5f,
    .kd = 0.5f,
};

void MotorThread_Init(CAN_HandleTypeDef *hcan, uint8_t motor_id)
{
    motor_hcan = hcan;
    motor_cmd.motor_id = motor_id;

    for (int i = 0; i < 5; ++i) {
        RS05_Private_Enable(motor_hcan, motor_cmd.motor_id);
        HAL_Delay(10);
    }
}

void MotorThread_SetTarget(float position, float speed, float kp, float kd, float torque)
{
    motor_cmd.target_position = position;
    motor_cmd.target_speed = speed;
    motor_cmd.kp = kp;
    motor_cmd.kd = kd;
    motor_cmd.target_torque = torque;
}

void MotorThread_Run10ms(void)
{
    if (motor_hcan == 0) {
        return;
    }

    MotorThread_SetTarget(
        g_debug_target_position,
        g_debug_target_speed,
        g_debug_kp,
        g_debug_kd,
        g_debug_target_torque
    );

    RS05_Private_Control(
        motor_hcan,
        motor_cmd.motor_id,
        motor_cmd.target_torque,
        motor_cmd.target_position,
        motor_cmd.target_speed,
        motor_cmd.kp,
        motor_cmd.kd
    );
}
