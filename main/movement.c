#include "motors.h"
#include "movement.h"
#include "pid.h"
#include "math.h"
#include "ble_telemetry.h"

#define MAX_RPM     210.0f
#define PI          3.14159f
static const float k = (30.0f * sqrt(2.0f) / (WHEEL_RADIUS_MM * PI));

typedef struct {
    float vx, vy;
} target_t;

static target_t target = {};

/* calculate RPMs to achieve desired forward and lateral velocities with mecanum wheels */
static void
calculate_rpms(float vx, float vy, float out_rpms[4])
{
    out_rpms[0] = (vx + vy) * k; // front right
    out_rpms[1] = (vx - vy) * k; // rear right
    out_rpms[2] = -1 * (vx - vy) * k; // front left
    out_rpms[3] = -1 * (vx + vy) * k; // rear left

    // compensation for our dodgy wheel. "fixing" mechanical problems in software is excellent.
    if (fabsf(vy) > 0.1f) {
        out_rpms[1] *= 1.0f + (2.0f * out_rpms[1] / MAX_RPM) * (out_rpms[1] > 0 ? 1.0f : -1.0f);
    }
}

/* set the desired forward/lateral velocities */
void
movement_set(float vx, float vy)
{
    target.vx = vx;
    target.vy = vy;
}

/* update the movement control system
 *
 * original idea was to use IMU readings to keep track of yaw and have a PID loop to vary
 * the robot's angular velocity so that it's always facing (approximately) the same direction.
 * this was not a focus after discovering the dodgy wheel
 */
void
movement_pid_update(int frequency_hz)
{
    float rpms[4] = {};
    calculate_rpms(target.vx, target.vy, rpms);

    for (int i = 0; i < 4; ++i) {
        telemetry.target_rpms[i] = rpms[i];
    }

    motors_set_rpms(rpms);
    motors_pid_update(frequency_hz);
}
