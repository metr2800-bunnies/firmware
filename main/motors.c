#include "motors.h"
#include "quadrature_encoder.h"
#include "tb6612fng.h"
#include "ble_telemetry.h"
#include "math.h"

#define TICKS_PER_REV       341.2f
#define ENCODER_RESOLUTION  ((TICKS_PER_REV) * 4)
#define MAX_INTEGRAL_ERROR  50.0f
#define DEADZONE_THRESHOLD  0.25f
#define MAX_RPM             210.0f
#define RPM_SMOOTHING_ALPHA 0.2f

#define KP  0.001f
#define KI  0.002f
#define KD  0.0005f

typedef struct {
    quadrature_encoder_handle_t encoder;
    tb6612fng_channel_handle_t driver;
    float target_rpm;
    int last_tick_count;
    float last_rpm;
    float integral_error;
    float last_error;
} motor_t;

static motor_t motors[4] = {};

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
}

void
motors_set_rpm(int motor_index, float rpm)
{
    if (motor_index < 0 || motor_index > 3) {
        return;
    }
    motors[motor_index].target_rpm = rpm;
}

void
motors_control_update(int motor_index, int frequency_hz)
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

    /* PID stuff */
    float error = motor->target_rpm - motor->last_rpm;
    if (fabsf(error) < 3.0f) {
        error = 0.0f;
    }
    float proportional = KP * error;

    motor->integral_error += error / frequency_hz;
    // anti-windup clamping
    if (motor->integral_error > MAX_INTEGRAL_ERROR) {
        motor->integral_error = MAX_INTEGRAL_ERROR;
    } else if (motor->integral_error < -MAX_INTEGRAL_ERROR) {
        motor->integral_error = -MAX_INTEGRAL_ERROR;
    }
    float integral = KI * motor->integral_error;

    float derivative = KD * (error - motor->last_error) * frequency_hz;
    motor->last_error = error;

    float feedforward = motor->target_rpm * 0.5 / MAX_RPM;
    float speed = feedforward + proportional + integral + derivative;

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
    telemetry.ff = feedforward;
    telemetry.p = proportional;
    telemetry.i = integral;
    telemetry.d = derivative;
}

float
motors_get_rpm(int motor_index)
{
    if (motor_index < 0 || motor_index > 3) {
        return 0;
    }
    return motors[motor_index].last_rpm;
}
