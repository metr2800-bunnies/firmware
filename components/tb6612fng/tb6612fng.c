#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY_HZ       2500 // note that motor driver has max switching frequency of 10kHz
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT
#define DUTY_RANGE              (1 << 10)

typedef struct {
    int in1_pin, in2_pin, speed_pin, ledc_channel;
    int mode;
    float speed;
} tb6612fng_channel_dev_t;

static const char *TAG = "TB6612FNG"

static int next_ledc_channel = 0;

esp_err_t
tb6612fng_init(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "failed to initialise LEDC timer");
}

esp_err_t
tb6612fng_channel_create(int in1_pin, int in2_pin, int speed_pin, tb6612fng_channel_handle_t *ret_handle)
{
    esp_err_t ret = ESP_OK;
    tb6612fng_channel_dev_t *channel = (tb6612fng_channel_dev_t *)calloc(1, sizeof(tb6612fng_channel_dev_t));
    ESP_GOTO_ON_FALSE(channel, ESP_ERR_NO_MEM, err, TAG, "no mem for TB6612FNG channel");

    channel->in1_pin = in1_pin;
    channel->in2_pin = in2_pin;
    channel->speed_pin = speed_pin;
    channel->ledc_channel = next_ledc_channel++;
    channel->mode = 0;
    channel->speed = 0.0f;

    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << in1_pin) | (1ULL << in2_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_config);
    ESP_GOTO_ON_ERROR(gpio_set_level(in1_pin, 1), err, TAG, "failed to set initial IN1 value");
    ESP_GOTO_ON_ERROR(gpio_set_level(in2_pin, 1), err, TAG, "failed to set initial IN2 value");

    ledc_channel_config_t channel_config = {
        .speed_mode = LEDC_MODE,
        .channel = channel->ledc_channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = speed_pin,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_GOTO_ON_ERROR(ledc_channel_config(&channel_config), err, TAG, "failed to intitialise LEDC channel");

    ESP_GOTO_ON_ERROR(tb6612fng_channel_set_drive(channel, TB6612FNG_STOP, 0.0f),
            err, TAG, "failed to stop motor channel");

    *ret_handle = channel;
    return ESP_OK;
err:
    if (channel) {
        free(channel);
    }
    *ret_handle = NULL;
    return ret;
}

esp_err_t
tb6612fng_channel_set_drive(tb6612fng_channel_handle_t handle, tb6612fng_channel_drive_t mode, float speed)
{
    tb6612fng_channel_dev_t *channel = (tb6612fng_channel_dev_t *)handle;
    if (channel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (speed < 0.0f || speed > 1.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t duty = (uint32_t)(speed * DUTY_RANGE);

    int in1, in2;
    switch (mode) {
        case TB6612FNG_SHORT_BRAKE: in1 = 1; in2 = 1; break;
        case TB6612FNG_CCW: in1 = 0; in2 = 1; break;
        case TB6612FNG_CW: in1 = 1; in2 = 0; break;
        case TB6612FNG_STOP: in1 = 0; in2 = 0; duty = DUTY_RANGE; speed = 1.0f; break; // STOP requires 100% duty
        default:
            return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_MODE, handle->ledc_channel, duty), TAG, "failed to set PWM duty cycle");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_MODE, handle->ledc_channel), TAG, "failed to update PWM duty cycle");
    handle->speed = speed;

    ESP_RETURN_ON_ERROR(gpio_set_level(in1_pin, in1), TAG, "failed to set IN1");
    ESP_RETURN_ON_ERROR(gpio_set_level(in2_pin, in2), TAG, "failed to set IN2");
    handle->mode = mode;

    return ESP_OK;
}

esp_err_t
tb6612fng_channel_get_drive(tb6612fng_channel_handle_t handle, tb6612fng_channel_drive_t *ret_mode, float *ret_speed)
{
    tb6612fng_channel_dev_t *channel = (tb6612fng_channel_dev_t *)handle;
    if (channel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *ret_mode = channel->mode;
    *ret_speed = channel->speed;
    return ESP_OK;
}
