#include "motors.h"
#include "movement.h"
#include "pid.h"
#include "imu.h"
#include "math.h"
#include "ble_telemetry.h"

#define PI                      3.14159f
#define RAD_PER_SECOND_TO_RPM   (30 / PI)

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
    float k = 4.0f / WHEEL_RADIUS_MM * RAD_PER_SECOND_TO_RPM;
    float rot_factor = (LENGTH_MM + WIDTH_MM) / 4.0f;
    out_rpms[0] = (vx + vy + omega * rot_factor) * k; // front right
    out_rpms[1] = (vx - vy + omega * rot_factor) * k; // rear right
    out_rpms[2] = -1 * (vx - vy - omega * rot_factor) * k; // front left
    out_rpms[3] = -1 * (vx + vy - omega * rot_factor) * k; // rear left
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
    if (go) {
        float yaw = imu_get_yaw();
        omega_correction = pid_compute(&pid, frequency_hz / 2, 0.0f, yaw);
        go ^= 1;
    }

    if (fabsf(target.vx) < 0.1f && fabsf(target.vy) < 0.1f) {
        omega_correction = 0.0f;
    }

    float rpms[4] = {};
    calculate_rpms(target.vx, target.vy, omega_correction, rpms);

    for (int i = 0; i < 4; ++i) {
        telemetry.target_rpms[i] = rpms[i];
    }

    motors_set_rpms(rpms);
    motors_pid_update(frequency_hz);
}
