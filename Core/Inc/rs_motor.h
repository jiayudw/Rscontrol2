#ifndef __RS_MOTOR_H
#define __RS_MOTOR_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS_MOTOR_P_MIN  -12.57f
#define RS_MOTOR_P_MAX   12.57f
#define RS_MOTOR_V_MIN  -50.0f
#define RS_MOTOR_V_MAX   50.0f
#define RS_MOTOR_KP_MIN   0.0f
#define RS_MOTOR_KP_MAX   500.0f
#define RS_MOTOR_KD_MIN   0.0f
#define RS_MOTOR_KD_MAX   5.0f
#define RS_MOTOR_T_MIN   -5.5f
#define RS_MOTOR_T_MAX    5.5f

typedef struct {
    uint8_t can_id;
    float position;
    float velocity;
    float torque;
    float temperature;
    uint8_t mode_state;
    uint8_t fault_code;
} RSMotorFeedback_t;

int RS_Motor_FloatToUint(float x, float x_min, float x_max, int bits);
float RS_Motor_UintToFloat(int x_int, float x_min, float x_max, int bits);
uint32_t RS_Motor_GetExtId(uint8_t mode, uint16_t data, uint8_t motor_id);
void RS_Motor_BuildEnableFrame(uint8_t id, CAN_TxHeaderTypeDef *header, uint8_t data[8]);
void RS_Motor_BuildDisableFrame(uint8_t id, CAN_TxHeaderTypeDef *header, uint8_t data[8]);
void RS_Motor_BuildControlFrame(uint8_t id, float torque, float position, float speed,
                                float kp, float kd, CAN_TxHeaderTypeDef *header, uint8_t data[8]);
int RS_Motor_ParseFeedback(uint32_t ext_id, const uint8_t data[8], RSMotorFeedback_t *feedback);

#ifdef __cplusplus
}
#endif

#endif
