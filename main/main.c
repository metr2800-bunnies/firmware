#include "driver/i2c_master.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "led_strip.h"
#include "mpu6050.h"
#include "motors.h"
#include "servos.h"
#include "movement.h"
#include "ble_telemetry.h"

/* I2C GPIO */
#define SDA_GPIO        GPIO_NUM_1
#define SCL_GPIO        GPIO_NUM_4
#define I2C_PORT_NUM    I2C_NUM_0

/* LIMIT SWITCH GPIO */
#define LIM1_GPIO   GPIO_NUM_45
#define LIM2_GPIO   GPIO_NUM_35
#define LIM3_GPIO   GPIO_NUM_9
#define LIM4_GPIO   GPIO_NUM_10

/* PUSHBUTTON GPIO */
#define BOOT_BUTTON_GPIO    GPIO_NUM_0

/* LED GPIO */
#define LED_GPIO            GPIO_NUM_38

#define TIMER_RESOLUTION_HZ     1000000
#define TIMER_FREQ_HZ           10

enum {
    IDLE = 0,
    GO,
    GO_TO_BALLS,
    WAIT_TO_GET_TO_BALLS,
    OPEN_BOX,
    WAIT_FOR_OPEN,
    LOWER_BOX,
    WAIT_FOR_LOWER,
    SCOOP_BALLS,
    WAIT_FOR_SCOOP,
    RETRACT_BOX,
    WAIT_FOR_RETRACT,
    RAISE_BOX,
    WAIT_FOR_RAISE,
    GO_OVER_SEESAW,
    WAIT_TO_GET_OVER_SEESAW,
    GO_TO_DEPOSIT,
    WAIT_TO_GET_TO_DEPOSIT,
    EJECT_BALLS,
    WAIT_FOR_EJECT,
    PARK,
    WAIT_FOR_PARK,
    STOP,
} state_t;

static SemaphoreHandle_t control_loop_semaphore;
static gptimer_handle_t gptimer = 0;
static led_strip_handle_t led_strip;

static void
configure_led(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
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
    servos_init();
    ble_telemetry_init();

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255, 0, 0));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    movement_set(0.0f, 0.0f, 0.0f);
    servo_set(0.0f, 0.0f, 0.0f);

    state_t state = IDLE;
    uint32_t ticks = 0;
    while (1) {
        switch (state) {
            case IDLE:
                if (!gpio_get_level(BOOT_BUTTON_GPIO)) {
                    state = GO;
                }
                break;
            case GO:
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 255, 0));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                state = GO_TOWARD_BALLS;
                break;
            case GO_TO_BALLS:
                movement_set(50.0f, 0.0f, 0.0f);
                servo_set(0.0f, 0.0f, 0.0f);
                ticks = 0;
                state = WAIT_TO_ARRIVE_AT_BALLS;
                break;
            case WAIT_TO_GET_TO_BALLS:
                if (ticks >= 10 * TIMER_FREQ_HZ) {
                    state = OPEN_BOX;
                }
                break;
            case OPEN_BOX:
                movement_set(0.0f, 0.0f, 0.0f);
                servo_set(1.0f, 1.0f, 0.0f);
                ticks = 0;
                state = WAIT_FOR_OPEN;
                break;
            case WAIT_FOR_OPEN:
                // todo: replace with limit switch
                if (ticks >= 2 * TIMER_FREQ_HZ) {
                    state = LOWER_BOX;
                }
                break;
            case LOWER_BOX:
                movement_set(0.0f, 0.0f, 0.0f);
                servo_set(0.0f, 0.0f, 1.0f);
                ticks = 0;
                break;
            case WAIT_FOR_LOWER:
                // todo: replace with limit switch
                if (ticks >= 5 * TIMER_FREQ_HZ) {
                    state = SCOOP_BALLS;
                }
                break;
            case SCOOP_BALLS:
                movement_set(0.0f, 20.0f, 0.0f);
                servo_set(0.0f, 0.0f, 0.0f);
                ticks = 0;
                break;
            case WAIT_FOR_SCOOP:
                if (ticks >= 1 * TIMER_FREQ_HZ) {
                    state = SCOOP_BALLS;
                }
                break;
            case RETRACT_BOX:
                movement_set(0.0f, 0.0f, 0.0f);
                servo_set(-1.0f, -1.0f, 0.0f);
                ticks = 0;
                break;
            case WAIT_FOR_RETRACT:
                // todo: replace with limit switch
                if (ticks >= 2 * TIMER_FREQ_HZ) {
                    state = SCOOP_BALLS;
                }
                break;
            case RAISE_BOX:
                movement_set(0.0f, 0.0f, 0.0f);
                servo_set(0.0f, 0.0f, -1.0f);
                ticks = 0;
                break;
            case WAIT_FOR_RAISE:
                // todo: replace with limit switch
                if (ticks >= 2 * TIMER_FREQ_HZ) {
                    state = SCOOP_BALLS;
                }
                break;
            case GO_OVER_SEESAW:
                movement_set(0.0f, 100.0f, 0.0f);
                servo_set(0.0f, 0.0f, 0.0f);
                ticks = 0;
                break;
            case WAIT_TO_GET_OVER_SEESAW:
                if (ticks >= 20 * TIMER_FREQ_HZ) {
                    state = SCOOP_BALLS;
                }
                break;
            case GO_TO_DEPOSIT:
                movement_set(-50.0f, 0.0f, 0.0f);
                servo_set(0.0f, 0.0f, 0.0f);
                ticks = 0;
                break;
            case WAIT_TO_GET_TO_DEPOSIT:
                if (ticks >= 2 * TIMER_FREQ_HZ) {
                    state = SCOOP_BALLS;
                }
                break;
            case EJECT_BALLS:
                movement_set(0.0f, 0.0f, 0.0f);
                servo_set(-1.0f, -1.0f, 0.0f);
                ticks = 0;
                break;
            case WAIT_FOR_EJECT:
                // todo: replace with limit switch
                if (ticks >= 2 * TIMER_FREQ_HZ) {
                    state = SCOOP_BALLS;
                }
                break;
            case PARK:
                movement_set(100.0f, 0.0f, 0.0f);
                servo_set(0.0f, 0.0f, 0.0f);
                ticks = 0;
                break;
            case WAIT_FOR_PARK:
                if (ticks >= 2 * TIMER_FREQ_HZ) {
                    state = SCOOP_BALLS;
                }
                break;
            case STOP:
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255, 0, 0));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                movement_set(0.0f, 0.0f, 0.0f);
                servo_set(0.0f, 0.0f, 0.0f);
                return;
        }
        movement_pid_update(TIMER_FREQ_HZ);
        ticks += 1;
        xSemaphoreTake(control_loop_semaphore, portMAX_DELAY);
    }
}
