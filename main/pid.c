#include "pid.h"
#include "math.h"

float
pid_compute(pid_t *pid, int update_frequency_hz, float target, float measured)
{
    float error = target - measured;
    if (fabsf(error) < pid->error_deadzone) {
        error = 0.0f;
    }

    float feedforward = pid->ff * target;
    float proportional = pid->kp * error;

    // integral with anti-windup
    pid->error_sum += error / update_frequency_hz;
    if (pid->error_sum > pid->max_error_sum) {
        pid->error_sum = pid->max_error_sum;
    } else if (pid->error_sum < -pid->max_error_sum) {
        pid->error_sum = -pid->max_error_sum;
    }
    float integral = pid->ki * pid->error_sum;

    float derivative = pid->kd * (error - pid->last_error) * update_frequency_hz;
    pid->last_error = error;

    return feedforward + proportional + integral + derivative;
}
