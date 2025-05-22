#include "motors.h"
#include "movement.h"
#include "pid.h"
#include "imu.h"
#include "math.h"
#include "ble_telemetry.h"

#define MAX_RPM     210.0f
#define PI          3.14159f
static const float k = (30.0f * sqrt(2.0f) / (WHEEL_RADIUS_MM * PI));

typedef struct {
    float vx, vy;
} target_t;

static target_t target = {};
static pid_t pid = {
    .ff = 0.0f,
    .kp = 0.5f,
    .ki = 0.01f,
    .kd = 0.05f,
    .error_sum = 0.0f,
    .last_error = 0.0f,
    .error_deadzone = 0.0f,
    .max_error_sum = 10.0f,
};

static void
calculate_rpms(float vx, float vy, float omega, float out_rpms[4])
{
    float rot_factor = (LENGTH_MM + WIDTH_MM) / 2.0f;
    out_rpms[0] = (vx + vy + omega * rot_factor) * k; // front right
    out_rpms[1] = (vx - vy + omega * rot_factor) * k; // rear right
    out_rpms[2] = -1 * (vx - vy - omega * rot_factor) * k; // front left
    out_rpms[3] = -1 * (vx + vy - omega * rot_factor) * k; // rear left

    if (fabsf(vy) > 0.1f) {
        out_rpms[1] *= 1.0f + (2.0f * out_rpms[1] / MAX_RPM) * (out_rpms[1] > 0 ? 1.0f : -1.0f);
    }
}

void
movement_set(float vx, float vy)
{
    target.vx = vx;
    target.vy = vy;
}

static float omega_correction = 0.0f;
static int go = 1;

void
movement_pid_update(int frequency_hz)
{
//    if (go) {
//        float yaw = imu_get_yaw();
//        omega_correction = pid_compute(&pid, frequency_hz / 2, 0.0f, yaw);
//        go ^= 1;
//    }
//
//    if (fabsf(target.vx) < 0.1f && fabsf(target.vy) < 0.1f) {
//        omega_correction = 0.0f;
//    }
//
    float rpms[4] = {};
    calculate_rpms(target.vx, target.vy, 0.0f, rpms);

    for (int i = 0; i < 4; ++i) {
        telemetry.target_rpms[i] = rpms[i];
    }

    motors_set_rpms(rpms);
    motors_pid_update(frequency_hz);
}
