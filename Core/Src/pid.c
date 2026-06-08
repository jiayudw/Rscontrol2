#include "pid.h"

static float Pid_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}

void Pid_Init(Pid_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    Pid_Reset(pid);
}

void Pid_Reset(Pid_t *pid)
{
    if (pid == 0) {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

float Pid_Update(Pid_t *pid, float target, float feedback, float dt_s)
{
    if (pid == 0) {
        return 0.0f;
    }

    if (dt_s <= 0.0f) {
        dt_s = 0.001f;
    }

    float error = target - feedback;
    pid->integral += error * dt_s;
    pid->integral = Pid_Clamp(pid->integral, -pid->integral_limit, pid->integral_limit);

    float derivative = (error - pid->last_error) / dt_s;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->last_error = error;

    return Pid_Clamp(output, -pid->output_limit, pid->output_limit);
}
