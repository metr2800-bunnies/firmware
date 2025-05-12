#include "motors.h"
#include "quadrature_encoder.h"
#include "tb6612fng.h"

typedef struct {
    quadrature_encoder_handle_t encoder;
    tb6612fng_channel_handle_t driver;
    float target_rpm;
    int last_tick_count;
    float last_rpm;
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
        ESP_LOGE("invalid motor index %d. ignoring.", motor_index);
        return;
    }
    motors[motor_index].target_rpm = rpm;
}

void
motors_control_update(int motor_index, int frequency_hz)
{
    if (motor_index < 0 || motor_index > 3) {
        ESP_LOGE("invalid motor index %d. ignoring.", motor_index);
        return;
    }
    motor_t *motor = motors[motor_index];

    /* turn encoder readings into RPM */

    float error = motor->target_rpm - motor->last_rpm;
    float proportional = Kp * error;
    float integral = 
}

float
motors_get_rpm(int motor_index)
{
    if (motor_index < 0 || motor_index > 3) {
        ESP_LOGE("invalid motor index %d. ignoring.", motor_index);
        return;
    }
    return motors[motor_index].last_rpm;
}
