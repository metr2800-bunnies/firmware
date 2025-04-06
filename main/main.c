#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "led_strip.h"
#include "quadrature_encoder.h"

/* ENCODER GPIO */
#define ENCODER1A_GPIO  GPIO_NUM_12
#define ENCODER1B_GPIO  GPIO_NUM_11
#define ENCODER2A_GPIO  GPIO_NUM_14
#define ENCODER2B_GPIO  GPIO_NUM_13
#define ENCODER3A_GPIO  GPIO_NUM_20
#define ENCODER3B_GPIO  GPIO_NUM_19
#define ENCODER4A_GPIO  GPIO_NUM_47
#define ENCODER4B_GPIO  GPIO_NUM_21

#define LED_GPIO            GPIO_NUM_38
#define LED_STRIP_LENGTH    1
#define LED_BRIGHTNESS      0.05    // neopixel LEDs are bright man

#define TIMER_RESOLUTION_HZ         1000000
#define TIMER_FREQ_HZ_NUMERATOR     5
#define TIMER_FREQ_HZ_DENOMINATOR   2

static gptimer_handle_t gptimer = 0;
static led_strip_handle_t led_strip;
static volatile uint8_t state = 0;

static void
configure_led(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_STRIP_LENGTH,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 0,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
}

static bool IRAM_ATTR
timer_interrupt(gptimer_handle_t timer,
        const gptimer_alarm_event_data_t *eventData, void *userData)
{
    state = (state + 1) % 3;
    return true;
}

static void
timer_setup(void)
{
    gptimer_config_t timerConfig = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &gptimer));

    gptimer_alarm_config_t alarmConfig = {
        .alarm_count = TIMER_RESOLUTION_HZ * TIMER_FREQ_HZ_DENOMINATOR / TIMER_FREQ_HZ_NUMERATOR,
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

static quadrature_encoder_handle_t encoders[4] = {};

static void
encoder_setup(void)
{
    quadrature_encoder_init();
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER1A_GPIO, ENCODER1B_GPIO, &encoders[0]));
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER2A_GPIO, ENCODER2B_GPIO, &encoders[0]));
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER3A_GPIO, ENCODER3B_GPIO, &encoders[0]));
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER4A_GPIO, ENCODER4B_GPIO, &encoders[0]));
}

void
app_main(void)
{
    configure_led();
    timer_setup();
    encoder_setup();

    uint8_t last_state = 255;

    while (1) {
        if (last_state != state) {
            last_state = state;
            uint8_t r, g, b;
            switch (state) {
                case 0: r = (uint8_t)(255 * LED_BRIGHTNESS); g = 0; b = 0; break;
                case 1: r = 0; g = (uint8_t)(255 * LED_BRIGHTNESS); b = 0; break;
                case 2: r = 0; g = 0; b = (uint8_t)(255 * LED_BRIGHTNESS); break;
                default: r = g = b = 0; break;
            }
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, b));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
