#include "driver/i2c_master.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "led_strip.h"
#include "mpu6050.h"
#include "motors.h"
#include "ble_telemetry.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* I2C GPIO */
#define SDA_GPIO        GPIO_NUM_1
#define SCL_GPIO        GPIO_NUM_4
#define I2C_PORT_NUM    I2C_NUM_0

/* LIMIT SWITCH GPIO */
#define LIM1_GPIO   GPIO_NUM_45
#define LIM2_GPIO   GPIO_NUM_35
#define LIM3_GPIO   GPIO_NUM_9
#define LIM4_GPIO   GPIO_NUM_10

/* SERVO GPIO */
#define SERVO1_GPIO     GPIO_NUM_18
#define SERVO2_GPIO     GPIO_NUM_8
#define SERVO3_GPIO     GPIO_NUM_46

/* PUSHBUTTON GPIO */
#define BOOT_BUTTON_GPIO    GPIO_NUM_0

/* LED GPIO */
#define LED_GPIO            GPIO_NUM_38
#define LED_STRIP_LENGTH    1
#define LED_BRIGHTNESS      0.05    // neopixel LEDs are bright man

#define TIMER_RESOLUTION_HZ     1000000
#define TIMER_FREQ_HZ           10

static const char *tag = "TABLEBOT";

static SemaphoreHandle_t control_loop_semaphore;
static gptimer_handle_t gptimer = 0;
static led_strip_handle_t led_strip;

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
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(control_loop_semaphore, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
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
        .alarm_count = TIMER_RESOLUTION_HZ / TIMER_FREQ_HZ,
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
    control_loop_semaphore = xSemaphoreCreateBinary();
    configure_led();
    timer_setup();
    motors_init();
    ble_telemetry_init();

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    int state = 0;
    while (1) {
        switch (state) {
            case 0:
                ESP_LOGI(tag, "Initialising");
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255, 0, 0));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                motors_set_rpm(0, 0);
                motors_set_rpm(1, 0);
                motors_set_rpm(2, 0);
                motors_set_rpm(3, 0);
                state = 1;
                break;
            case 1:
                if (!gpio_get_level(BOOT_BUTTON_GPIO)) {
                    ESP_LOGI(tag, "Button pushed");
                    state = 2;
                }
                break;
            case 2:
                ESP_LOGI(tag, "Setting target RPM");
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 255, 0));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                motors_set_rpm(0, 30);
                motors_set_rpm(1, 30);
                motors_set_rpm(2, -30);
                motors_set_rpm(3, -30);
                state = 3;
                break;
            case 3:
                motors_control_update(0, TIMER_FREQ_HZ);
                motors_control_update(1, TIMER_FREQ_HZ);
                motors_control_update(2, TIMER_FREQ_HZ);
                motors_control_update(3, TIMER_FREQ_HZ);
                break;
        }
        xSemaphoreTake(control_loop_semaphore, portMAX_DELAY);
    }
}
