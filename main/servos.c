#include "servos.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define SERVO1_GPIO     GPIO_NUM_18
#define SERVO2_GPIO     GPIO_NUM_8
#define SERVO3_GPIO     GPIO_NUM_46

#define LEDC_TIMER              LEDC_TIMER_2
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY_HZ       50
#define LEDC_DUTY_RES           LEDC_TIMER_12_BIT
#define DUTY_RANGE              (1 << 12)

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
        .pin_bit_mask = (1ULL << SERVO1_GPIO) | (1ULL << SERVO2_GPIO) | (1ULL << SERVO3_GPIO),
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
            .channel = 5 + i,
            .timer_sel = LEDC_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = gpios[i],
            .duty = 0,
            .hpoint = 0,
        };
        ledc_channel_config(&channel_config);
        ledc_set_duty(LEDC_MODE, 5 + i, DUTY_RANGE / 2);
        ledc_update_duty(LEDC_MODE, 5 + i);
    }
}

static void
set(int servo, float x)
{
    if (x > 1.0f || x < -1.0f) {
        return;
    }
    float pulse_width = 0.075f + (x * 0.025); // map [-1,1] onto [0.05,0.10]
    ledc_set_duty(LEDC_MODE, servo, (uint32_t)(pulse_width * DUTY_RANGE));
    ledc_update_duty(LEDC_MODE, servo);
}

void
servo_set(float a, float b, float c)
{
    set(5, a);
    set(6, b);
    set(7, c);
}
