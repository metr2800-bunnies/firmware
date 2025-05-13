#include "motors.h"
#include "quadrature_encoder.h"
#include "tb6612fng.h"
#include "ble_telemetry.h"

#define TICKS_PER_REV       341.2f
#define ENCODER_RESOLUTION  ((TICKS_PER_REV) * 4)
#define MAX_INTEGRAL_ERROR  50.0f

#define KP  0.8
#define KI  1.2
#define KD  0.05

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
        //ESP_LOGE("invalid motor index %d. ignoring.", motor_index);
        return;
    }
    motors[motor_index].target_rpm = rpm;
}

void
motors_control_update(int motor_index, int frequency_hz)
{
    if (motor_index < 0 || motor_index > 3) {
        //ESP_LOGE("invalid motor index %d. ignoring.", motor_index);
        return;
    }
    motor_t *motor = &motors[motor_index];

    /* turn encoder readings into RPM */
    int32_t count = motor->last_tick_count;
    quadrature_encoder_get_count(motor->encoder, &count);
    int32_t delta = count - motor->last_tick_count;
    motor->last_tick_count = count;
    motor->last_rpm = delta / ENCODER_RESOLUTION * frequency_hz * 60;

    telemetry.encoder_counts[motor_index] = count;
    telemetry.rpms[motor_index] = motor->last_rpm;

    /* PID stuff */
    float error = motor->target_rpm - motor->last_rpm;
    float proportional = KP * error;

    motor->integral_error += error / frequency_hz;
    if (motor->integral_error > MAX_INTEGRAL_ERROR) {
        motor->integral_error = MAX_INTEGRAL_ERROR;
    } else if (motor->integral_error < -MAX_INTEGRAL_ERROR) {
        motor->integral_error = -MAX_INTEGRAL_ERROR;
    }
    float integral = KI * motor->integral_error;

    float derivative = KD * (error - motor->last_error) * frequency_hz;
    motor->last_error = error;

    tb6612fng_channel_drive_t mode = TB6612FNG_STOP;
    float speed = 0.0f;
    tb6612fng_channel_get_drive(motor->driver, &mode, &speed);
    if (mode == TB6612FNG_CCW) {
        speed = -speed;
    }

    speed = speed + proportional + integral + derivative;

    telemetry.drive_power[motor_index] = speed;

    /* make sure values are sane */
    if (speed < 0) {
        speed = -speed;
        mode = TB6612FNG_CCW;
    } else {
        mode = TB6612FNG_CW;
    }

    if (speed < 1e-6) {
        speed = 0;
        mode = TB6612FNG_STOP;
    } else if (speed > 1.0f) {
        speed = 1.0f;
    }

    tb6612fng_channel_set_drive(motor->driver, mode, speed);
}

float
motors_get_rpm(int motor_index)
{
    if (motor_index < 0 || motor_index > 3) {
        //ESP_LOGE("invalid motor index %d. ignoring.", motor_index);
        return 0;
    }
    return motors[motor_index].last_rpm;
}
