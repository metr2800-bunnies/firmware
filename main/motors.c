#include "motors.h"
#include "ble_telemetry.h"
#include "math.h"
#include "quadrature_encoder.h"
#include "tb6612fng.h"
#include "pid.h"

#define TICKS_PER_REV       (34*11)
#define ENCODER_RESOLUTION  ((TICKS_PER_REV) * 4)
#define MAX_ERROR_SUM       10.0f
#define MAX_RPM             210.0f
#define DEADZONE_THRESHOLD  0.20f
#define RPM_SMOOTHING_ALPHA 0.2f
#define ERROR_DEADZONE      3.0f

#define FF  (0.5 * (MAX_RPM))
#define KP  0.001f
#define KI  0.003f
#define KD  0.0005f

typedef struct {
    quadrature_encoder_handle_t encoder;
    tb6612fng_channel_handle_t driver;
    float target_rpm, last_rpm;
    int last_tick_count;
    pid_t pid;
} motor_t;

static motor_t motors[4] = {};

static void
reset_parameters(void)
{
    for (int i = 0; i < 4; ++i) {
        int32_t count = 0;
        quadrature_encoder_get_count(motors[i].encoder, &count);

        motors[i].target_rpm = 0;
        motors[i].last_rpm = 0;
        motors[i].last_tick_count = count;
        motors[i].pid.ff = FF;
        motors[i].pid.kp = KP;
        motors[i].pid.ki = KI;
        motors[i].pid.kd = KD;
        motors[i].pid.error_sum = 0.0f;
        motors[i].pid.last_error = 0.0f;
        motors[i].pid.error_deadzone = ERROR_DEADZONE;
        motors[i].pid.max_error_sum = MAX_ERROR_SUM;
    }
}

void
motors_init(void)
{
    /* encoders */
    ESP_ERROR_CHECK(quadrature_encoder_init());
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER1A_GPIO, ENCODER1B_GPIO, &motors[0].encoder));
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER2A_GPIO, ENCODER2B_GPIO, &motors[1].encoder));
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER3A_GPIO, ENCODER3B_GPIO, &motors[2].encoder));
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER4A_GPIO, ENCODER4B_GPIO, &motors[3].encoder));

    /* motors */
    ESP_ERROR_CHECK(tb6612fng_init());
    ESP_ERROR_CHECK(tb6612fng_channel_create(MOTOR1_IN1_GPIO, MOTOR1_IN2_GPIO, MOTOR1_PWM_GPIO, &motors[0].driver));
    ESP_ERROR_CHECK(tb6612fng_channel_create(MOTOR2_IN1_GPIO, MOTOR2_IN2_GPIO, MOTOR2_PWM_GPIO, &motors[1].driver));
    ESP_ERROR_CHECK(tb6612fng_channel_create(MOTOR3_IN1_GPIO, MOTOR3_IN2_GPIO, MOTOR3_PWM_GPIO, &motors[2].driver));
    ESP_ERROR_CHECK(tb6612fng_channel_create(MOTOR4_IN1_GPIO, MOTOR4_IN2_GPIO, MOTOR4_PWM_GPIO, &motors[3].driver));

    reset_parameters();
}

void
motors_set_rpms(float rpms[4])
{
    for (int i = 0; i < 4; ++i) {
        motors[i].target_rpm = rpms[i];
    }
}

static void
motor_update(int motor_index, int frequency_hz)
{
    if (motor_index < 0 || motor_index > 3) {
        return;
    }
    motor_t *motor = &motors[motor_index];

    /* turn encoder readings into RPM */
    int32_t count = 0;
    quadrature_encoder_get_count(motor->encoder, &count);
    int32_t delta = count - motor->last_tick_count;
    motor->last_tick_count = count;

    float raw_rpm = (delta * frequency_hz * 60) / ENCODER_RESOLUTION;
    motor->last_rpm = (
        motor->last_rpm * (1.0f - RPM_SMOOTHING_ALPHA) +
        raw_rpm * RPM_SMOOTHING_ALPHA
    );

    /* some telemetry */
    telemetry.encoder_counts[motor_index] = count;
    telemetry.rpms[motor_index] = motor->last_rpm;

    float speed = pid_compute(&motor->pid, frequency_hz, motor->target_rpm, motor->last_rpm);

    // clamp
    if (speed > 1.0f) {
        speed = 1.0f;
    } else if (speed < -1.0f) {
        speed = -1.0f;
    }

    // deadzone to avoid oscillation near zero
    tb6612fng_channel_drive_t mode = TB6612FNG_STOP;
    if (speed > 0.0f) {
        mode = TB6612FNG_CW;
        speed = fminf(1.0f, fmaxf(speed, DEADZONE_THRESHOLD));
    } else if (speed < 0.0f) {
        mode = TB6612FNG_CCW;
        speed = fminf(1.0f, fmaxf(-speed, DEADZONE_THRESHOLD));
    } else {
        mode = TB6612FNG_STOP;
        speed = 0.0f;
    }
    tb6612fng_channel_set_drive(motor->driver, mode, speed);
    telemetry.drive_power[motor_index] = speed;
}

void
motors_pid_update(int frequency_hz)
{
    for (int i = 0; i < 4; ++i) {
        motor_update(i, frequency_hz);
    }
}
