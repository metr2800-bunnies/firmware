#include "servos.h"

#define SERVO1_GPIO     GPIO_NUM_18
#define SERVO2_GPIO     GPIO_NUM_8
#define SERVO3_GPIO     GPIO_NUM_46

#define LEDC_TIMER              LEDC_TIMER_2
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY_HZ       2500 // note that motor driver has max switching frequency of 10kHz
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT
#define DUTY_RANGE              (1 << 10)

void
servo_init(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_config);

    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << in1_pin) | (1ULL << in2_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_config);

    int gpios[3] = { SERVO1_GPIO, SERVO2_GPIO, SERVO3_GPIO };

    for (int i = 0; i < 3; ++i) {
        ledc_channel_config_t channel_config = {
            .speed_mode = LEDC_MODE,
            .channel = gpios[i],
            .timer_sel = LEDC_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = ,
            .duty = 0,
            .hpoint = 0,
        };
        ledc_channel_config(&channel_config);
        ledc_set_duty(LEDC_MODE, gpios[i], DUTY_RANGE / 2);
        ledc_update_duty(LEDC_MODE, gpios[i]);
    }
}

void
servo_set(float a, float b, float c)
{
    ledc_set_duty(LEDC_MODE, SERVO1_GPIO, (uint32_t)(a * DUTY_RANGE));
    ledc_update_duty(LEDC_MODE, SERVO1_GPIO);
    ledc_set_duty(LEDC_MODE, SERVO2_GPIO, (uint32_t)(b * DUTY_RANGE));
    ledc_update_duty(LEDC_MODE, SERVO2_GPIO);
    ledc_set_duty(LEDC_MODE, SERVO3_GPIO, (uint32_t)(c * DUTY_RANGE));
    ledc_update_duty(LEDC_MODE, SERVO3_GPIO);
}
