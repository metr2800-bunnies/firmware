#include "driver/i2c_master.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "led_strip.h"
#include "quadrature_encoder.h"
#include "tb6612fng.h"
#include "mpu6050.h"

/* ENCODER GPIO */
#define ENCODER1A_GPIO  GPIO_NUM_12
#define ENCODER1B_GPIO  GPIO_NUM_11
#define ENCODER2A_GPIO  GPIO_NUM_14
#define ENCODER2B_GPIO  GPIO_NUM_13
#define ENCODER3A_GPIO  GPIO_NUM_47
#define ENCODER3B_GPIO  GPIO_NUM_21
#define ENCODER4A_GPIO  GPIO_NUM_43
#define ENCODER4B_GPIO  GPIO_NUM_44

/* MOTOR GPIO */
#define MOTOR1_IN1_GPIO GPIO_NUM_7
#define MOTOR1_IN2_GPIO GPIO_NUM_6
#define MOTOR1_PWM_GPIO GPIO_NUM_5
#define MOTOR2_IN1_GPIO GPIO_NUM_15
#define MOTOR2_IN2_GPIO GPIO_NUM_16
#define MOTOR2_PWM_GPIO GPIO_NUM_17
#define MOTOR3_IN1_GPIO GPIO_NUM_39
#define MOTOR3_IN2_GPIO GPIO_NUM_37
#define MOTOR3_PWM_GPIO GPIO_NUM_36
#define MOTOR4_IN1_GPIO GPIO_NUM_40
#define MOTOR4_IN2_GPIO GPIO_NUM_41
#define MOTOR4_PWM_GPIO GPIO_NUM_42

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

#define TIMER_RESOLUTION_HZ         1000000
#define TIMER_FREQ_HZ_NUMERATOR     1
#define TIMER_FREQ_HZ_DENOMINATOR   4

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
i2c_setup(void)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_NUM, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_NUM, config.mode, 0, 0, 0));
}

mpu6050_handle_t mpu6050 = NULL;

static void
mpu6050_setup(void)
{
    // this other library (https://github.com/PiotrTopa/esp32-MPU6050) seems to support using DMP, which would be
    // kinda nice if we hadn't run out of GPIOs already.

    mpu6050 = mpu6050_create(I2C_PORT_NUM, MPU6050_I2C_ADDRESS);
    ESP_ERROR_CHECK(mpu6050_config(mpu6050, ACCE_FS_16G, GYRO_FS_2000DPS)); // probably don't need full range

    uint8_t id = 0;
    ESP_ERROR_CHECK(mpu6050_get_deviceid(mpu6050, &id));

    if (id == MPU6050_WHO_AM_I_VAL) {
        ESP_LOGI("main", "ID checks out");
    } else {
        ESP_LOGE("main", "Bad ID: %d", (int)id);
    }
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
    ESP_ERROR_CHECK(quadrature_encoder_init());
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER1A_GPIO, ENCODER1B_GPIO, &encoders[0]));
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER2A_GPIO, ENCODER2B_GPIO, &encoders[1]));
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER3A_GPIO, ENCODER3B_GPIO, &encoders[2]));
    ESP_ERROR_CHECK(quadrature_encoder_create(ENCODER4A_GPIO, ENCODER4B_GPIO, &encoders[3]));
}

static tb6612fng_channel_handle_t motors[4] = {};

static void
motor_setup(void)
{
    ESP_ERROR_CHECK(tb6612fng_init());
    ESP_ERROR_CHECK(tb6612fng_channel_create(MOTOR1_IN1_GPIO, MOTOR1_IN2_GPIO, MOTOR1_PWM_GPIO, &motors[0]));
    ESP_ERROR_CHECK(tb6612fng_channel_create(MOTOR2_IN1_GPIO, MOTOR2_IN2_GPIO, MOTOR2_PWM_GPIO, &motors[1]));
    ESP_ERROR_CHECK(tb6612fng_channel_create(MOTOR3_IN1_GPIO, MOTOR3_IN2_GPIO, MOTOR3_PWM_GPIO, &motors[2]));
    ESP_ERROR_CHECK(tb6612fng_channel_create(MOTOR4_IN1_GPIO, MOTOR4_IN2_GPIO, MOTOR4_PWM_GPIO, &motors[3]));
}

void
app_main(void)
{
    configure_led();
    timer_setup();
    encoder_setup();
    motor_setup();
    i2c_setup();
    mpu6050_setup();

    tb6612fng_channel_set_drive(motors[0], TB6612FNG_CCW, 0.2f);
    tb6612fng_channel_set_drive(motors[1], TB6612FNG_CW, 0.4f);
    tb6612fng_channel_set_drive(motors[2], TB6612FNG_CCW, 0.6f);
    tb6612fng_channel_set_drive(motors[3], TB6612FNG_CW, 0.8f);

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
