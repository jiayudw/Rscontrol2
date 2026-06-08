#ifndef __PID_H
#define __PID_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float integral;
    float last_error;
} Pid_t;

void Pid_Init(Pid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
void Pid_Reset(Pid_t *pid);
float Pid_Update(Pid_t *pid, float target, float feedback, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
