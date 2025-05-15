#pragma once

typedef struct {
    float ff, kp, ki, kd;
    float error_sum, last_error;
    float error_deadzone, max_error_sum;
} pid_t;

float pid_compute(pid_t *pid, int update_frequency_hz, float desired, float measured);
