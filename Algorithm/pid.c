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

void Pid_Init(PidController *pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_min, float integral_max)
{
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    Pid_Reset(pid);
}

void Pid_Reset(PidController *pid)
{
    if (pid == 0) {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

float Pid_Update(PidController *pid, float target, float feedback, float dt_s)
{
    float error;
    float derivative;
    float output;

    if (pid == 0) {
        return 0.0f;
    }

    if (dt_s <= 0.0f) {
        dt_s = 0.001f;
    }

    error = target - feedback;

    /* 积分项用于消除稳态误差；先限幅，避免长时间误差导致积分过大。 */
    pid->integral += error * dt_s;
    pid->integral = Pid_Clamp(pid->integral, pid->integral_min, pid->integral_max);

    derivative = (error - pid->last_error) / dt_s;
    pid->last_error = error;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return Pid_Clamp(output, pid->output_min, pid->output_max);
}
