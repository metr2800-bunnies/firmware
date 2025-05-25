#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "led_strip.h"
#include "motors.h"
#include "servos.h"
#include "movement.h"
#include "ble_telemetry.h"

/* LIMIT SWITCH GPIO
 *
 * the limit switches are active high and should use internal pulldown resistors
 */
#define LIM1_GPIO   GPIO_NUM_45
#define LIM2_GPIO   GPIO_NUM_35
#define LIM3_GPIO   GPIO_NUM_9
#define LIM4_GPIO   GPIO_NUM_10

#define LIMIT_SWITCH_GPIO_MASK  (1ULL << LIM1_GPIO || 1ULL << LIM2_GPIO || 1ULL << LIM3_GPIO || 1ULL << LIM4_GPIO)

#define SCISSOR_LIFT_TOP_LIM_GPIO       LIM3_GPIO
#define SCISSOR_LIFT_BOTTOM_LIM_GPIO    LIM1_GPIO

/* PUSHBUTTON GPIO
 *
 * the boot button is active low and should use internal pullup resistor
 */
#define BOOT_BUTTON_GPIO    GPIO_NUM_0

/* LED GPIO
 *
 * the single addressable LED included on this board
 */
#define LED_GPIO            GPIO_NUM_38

/* CONTROL FSM TIMER CONFIG
 *
 * these parameters are for the timer interrupt used to ensure robot state updates occur at a fixed rate
 */
#define TIMER_RESOLUTION_HZ     1000000
#define TIMER_FREQ_HZ           10

/* CONTROL FSM - ROBOT STATES */
typedef enum {
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
    GET_ON_RAMP,
    WAIT_TO_GET_ON_RAMP,
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

static SemaphoreHandle_t control_loop_semaphore; // given from timer interrupt ISR, taken in robot control FSM
static gptimer_handle_t gptimer = 0; // for the FSM interrupt
static led_strip_handle_t led_strip; // for the on-board LED

/* configures the on-board LED
 *
 * kinda annoying that it's an addressable LED and not just a regular one, more painful to use. RGB is cool though.
 */
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

/* the timer interrupt called at a rate of TIMER_FREQ_HZ. don't do any actual work here, just use semaphore */
static bool IRAM_ATTR
timer_interrupt(gptimer_handle_t timer,
        const gptimer_alarm_event_data_t *eventData, void *userData)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(control_loop_semaphore, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
    return true;
}

/* this is pretty self explanatory */
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
    /* initialise peripherals, GPIO, ... */
    control_loop_semaphore = xSemaphoreCreateBinary();
    configure_led();
    timer_setup();

    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255, 255, 255));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

    motors_init();
    movement_set(0.0f, 0.0f);

    servo_init();
    servo_pinion(0.0f);
    servo_winch(0.0f);

    ble_telemetry_init();

    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 0, 255));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_config_t io_conf_2 = {
        .pin_bit_mask = LIMIT_SWITCH_GPIO_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_2);

    /* raise scissor lift until it reaches the top. avoids stalling servo too much */
    servo_winch(0.7f);
    while (gpio_get_level(SCISSOR_LIFT_TOP_LIM_GPIO) != 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    servo_winch(0.0f);

    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255, 0, 0));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

    state_t state = IDLE;
    uint32_t ticks = 0; // incremented every iteration, used as a delay for timed state transitions
    int keep_winch_at_top = 1;
    while (1) {
        /* our servo can't quite keep box at top unless we stall it, which
         * is obviously not very good for it. if the keep_winch_at_top
         * flag is set, we turn on the servo if the scissor lift isn't
         * fully raised. this results in it bobbing up and down near the top
         * position, which is satisfactory (and seems to be less bad for it)
         */
        if (keep_winch_at_top) {
            if (gpio_get_level(SCISSOR_LIFT_TOP_LIM_GPIO) != 1) {
                servo_winch(0.5f);
            } else {
                servo_winch(0.0f);
            }
        }

        /* control FSM. nothing too exciting here */
        switch (state) {
            case IDLE:
                if (!gpio_get_level(BOOT_BUTTON_GPIO)) {
                    state = GO;
                }
                break;
            case GO:
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 255, 0));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                movement_set(0.0f, 0.0f);
                state = GO_TO_BALLS;
                break;
            case GO_TO_BALLS:
                movement_set(0.0f, 200.0f);
                ticks = 0;
                state = WAIT_TO_GET_TO_BALLS;
                break;
            case WAIT_TO_GET_TO_BALLS:
                if (ticks >= 12) {
                    state = OPEN_BOX;
                }
                break;
            case OPEN_BOX:
                movement_set(0.0f, 0.0f);
                servo_pinion(-0.7f);
                ticks = 0;
                state = WAIT_FOR_OPEN;
                break;
            case WAIT_FOR_OPEN:
                // todo: replace with limit switch
                if (ticks >= 30) {
                    state = LOWER_BOX;
                }
                break;
            case LOWER_BOX:
                movement_set(0.0f, 0.0f);
                servo_pinion(0.0f);
                keep_winch_at_top = 0;
                servo_winch(-0.2f);
                state = WAIT_FOR_LOWER;
                break;
            case WAIT_FOR_LOWER:
                if (gpio_get_level(SCISSOR_LIFT_BOTTOM_LIM_GPIO) == 1) {
                    state = SCOOP_BALLS;
                }
                break;
            case SCOOP_BALLS:
                movement_set(100.0f, 0.0f);
                ticks = 0;
                state = WAIT_FOR_SCOOP;
                break;
            case WAIT_FOR_SCOOP:
                if (ticks >= 15) {
                    state = RETRACT_BOX;
                }
                break;
            case RETRACT_BOX:
                movement_set(0.0f, 0.0f);
                servo_pinion(0.7f);
                ticks = 0;
                state = WAIT_FOR_RETRACT;
                break;
            case WAIT_FOR_RETRACT:
                // todo: replace with limit switch
                if (ticks >= 30) {
                    state = RAISE_BOX;
                }
                break;
            case RAISE_BOX:
                movement_set(0.0f, 0.0f);
                servo_pinion(0.0f);
                servo_winch(0.5f);
                state = WAIT_FOR_RAISE;
                break;
            case WAIT_FOR_RAISE:
                if (gpio_get_level(SCISSOR_LIFT_TOP_LIM_GPIO) == 1) {
                    state = GET_ON_RAMP;
                }
                break;
            case GET_ON_RAMP:
                keep_winch_at_top = 1;
                movement_set(400.0f, 0.0f);
                servo_pinion(0.0f);
                servo_winch(0.0f);
                ticks = 0;
                state = WAIT_TO_GET_ON_RAMP;
                break;
            case WAIT_TO_GET_ON_RAMP:
                if (ticks >= 15) {
                    state = GO_OVER_SEESAW;
                }
                break;
            case GO_OVER_SEESAW:
                movement_set(200.0f, 0.0f);
                ticks = 0;
                state = WAIT_TO_GET_OVER_SEESAW;
                break;
            case WAIT_TO_GET_OVER_SEESAW:
                if (ticks >= 80) {
                    state = GO_OVER_SEESAW;
                }
                break;
            case GO_TO_DEPOSIT:
                movement_set(0.0f, -50.0f);
                servo_pinion(0.0f);
                servo_winch(0.0f);
                ticks = 0;
                state = WAIT_TO_GET_TO_DEPOSIT;
                break;
            case WAIT_TO_GET_TO_DEPOSIT:
                if (ticks >= 2 * TIMER_FREQ_HZ) {
                    state = EJECT_BALLS;
                }
                break;
            case EJECT_BALLS:
                movement_set(0.0f, 0.0f);
                servo_pinion(1.0f);
                ticks = 0;
                state = WAIT_FOR_EJECT;
                break;
            case WAIT_FOR_EJECT:
                // todo: replace with limit switch
                if (ticks >= 30) {
                    state = PARK;
                }
                break;
            case PARK:
                //movement_set(0.0f, 100.0f);
                servo_pinion(-1.0f);
                ticks = 0;
                state = WAIT_FOR_PARK;
                break;
            case WAIT_FOR_PARK:
                if (ticks >= 35) {
                    state = STOP;
                }
                break;
            case STOP:
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255, 0, 0));
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                movement_set(0.0f, 0.0f);
                servo_pinion(0.0f);
                servo_winch(0.0f);
                return;
        }
        movement_pid_update(TIMER_FREQ_HZ);
        ticks += 1;
        xSemaphoreTake(control_loop_semaphore, portMAX_DELAY);
    }
}
