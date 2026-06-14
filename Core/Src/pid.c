#include "pid.h"

/* 将数值限制在给定上下限之间。 */
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

/* 初始化 PID 参数并清空历史状态。 */
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

/* 清空 PID 的积分项和上一拍误差。 */
void Pid_Reset(Pid_t *pid)
{
    if (pid == 0) {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

/* 根据目标值、反馈值和周期时间计算 PID 输出。 */
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
    /* 限制积分项，防止长时间大误差导致积分饱和。 */
    pid->integral = Pid_Clamp(pid->integral, -pid->integral_limit, pid->integral_limit);

    float derivative = (error - pid->last_error) / dt_s;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->last_error = error;

    /* 最终输出也限幅，避免给电机过大的电流/力矩命令。 */
    return Pid_Clamp(output, -pid->output_limit, pid->output_limit);
}
