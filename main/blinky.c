#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define LED_GPIO GPIO_NUM_2

/* timer config constants */
#define TIMER_RESOLUTION_HZ         1000000
#define TIMER_FREQ_HZ_NUMERATOR     5
#define TIMER_FREQ_HZ_DENOMINATOR   2

static gptimer_handle_t gptimer = 0;
volatile uint8_t state = 0;

static bool
IRAM_ATTR timer_interrupt(gptimer_handle_t timer,
        const gptimer_alarm_event_data_t *eventData, void *userData)
{
    state ^= 1;
    gpio_set_level(LED_GPIO, state);
    return true;
}

void
timer_setup(void)
{
    gptimer_config_t timerConfig = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &gptimer));

    gptimer_alarm_config_t alarmConfig = {
        .alarm_count = TIMER_RESOLUTION_HZ * TIMER_FREQ_HZ_DENOMINATOR
            / TIMER_FREQ_HZ_NUMERATOR,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarmConfig));

    gptimer_event_callbacks_t callbacks = {
        .on_alarm = timer_interrupt,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &callbacks, 0));

    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

void
app_main(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    timer_setup();

    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
