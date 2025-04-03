#include "driver/timer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define LED_GPIO GPIO_NUM_2

/* timer interrupt configuration constants */
#define CPU_TIMER_FREQ_HZ           80000000
#define TIMER_DIVIDER               80
#define TIMER_FREQ_HZ_NUMERATOR     1
#define TIMER_FREQ_HZ_DENOMINATOR   2

volatile uint8_t state = 0;

bool IRAM_ATTR timer_interrupt(void* arg) {
    state ^= 1;
    gpio_set_level(LED_GPIO, state);
    return true;
}

void timer_setup() {
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = TIMER_DIVIDER
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    uint64_t alarmValue = ((CPU_TIMER_FREQ_HZ / TIMER_DIVIDER)
            * TIMER_FREQ_HZ_DENOMINATOR / TIMER_FREQ_HZ_NUMERATOR);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, alarmValue);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_interrupt, NULL, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void app_main() {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    timer_setup();

    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
