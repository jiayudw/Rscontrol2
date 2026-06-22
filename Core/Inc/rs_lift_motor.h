#ifndef __RS_LIFT_MOTOR_H
#define __RS_LIFT_MOTOR_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t id;
    int8_t direction;
    uint8_t enabled;

    float target_speed_rad_s;
    float target_position_rad;

    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;

    uint8_t mode_state;
    uint8_t fault_code;
    uint8_t online;
    uint32_t last_update_ms;
} RsLiftMotor_t;

void RsLiftMotor_Init(CAN_HandleTypeDef *hcan);
void RsLiftMotor_SetCommand(int8_t lift_cmd);
void RsLiftMotor_Stop(void);
void RsLiftMotor_RunControl(void);
void RsLiftMotor_OnCanFeedback(uint32_t ext_id, uint8_t data[8]);
const RsLiftMotor_t *RsLiftMotor_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
