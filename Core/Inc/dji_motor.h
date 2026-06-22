#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

#include "main.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DJI_MOTOR_COUNT 4U
#define DJI_MOTOR_CAN_ID_MIN 0x201U
#define DJI_MOTOR_CAN_ID_MAX 0x208U

typedef struct {
    uint8_t id;
    int8_t direction;
    uint16_t ecd;
    uint16_t last_ecd;
    int16_t speed_rpm;
    int16_t current_raw;
    uint8_t temperature;
    int32_t round_count;
    float total_angle_deg;
    float target_rpm;
    int16_t output_current;
    uint8_t online;
    uint32_t last_update_ms;
    Pid_t speed_pid;
} DjiMotor_t;

void DjiMotor_Init(CAN_HandleTypeDef *hcan);
void DjiMotor_SetTargetRpm(uint8_t index, float target_rpm);
void DjiMotor_StopAll(void);
void DjiMotor_RunControl(float dt_s);
void DjiMotor_OnCanFeedback(uint32_t std_id, const uint8_t data[8]);
const DjiMotor_t *DjiMotor_GetState(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif
