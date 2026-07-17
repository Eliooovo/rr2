#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
} PidController;

void Pid_Init(PidController *pid, float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_min, float integral_max);
void Pid_Reset(PidController *pid);
float Pid_Update(PidController *pid, float target, float feedback, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
