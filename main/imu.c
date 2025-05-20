#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "ble_telemetry.h"

#define SDA_GPIO        GPIO_NUM_1
#define SCL_GPIO        GPIO_NUM_4
#define I2C_PORT_NUM    I2C_NUM_0

#define TIMER_RESOLUTION_HZ     1000000
#define TIMER_FREQ_HZ           100

static mpu6050_handle_t mpu6050 = NULL;
static SemaphoreHandle_t imu_semaphore;
static gptimer_handle_t gptimer = 0;

typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
} mpu6050_bias_t;

static mpu6050_bias_t sensor_bias = {0};

void mpu6050_calibrate(void) {
    const int samples = 50;
    float accel_sum[3] = {0};
    float gyro_sum[3] = {0};

    ESP_LOGI("imu", "Starting calibration. Keep robot stationary...");

    for (int i = 0; i < samples; i++) {
        mpu6050_acce_value_t accel;
        mpu6050_gyro_value_t gyro;

        // Read raw data
        ESP_ERROR_CHECK(mpu6050_get_acce(mpu6050, &accel));
        ESP_ERROR_CHECK(mpu6050_get_gyro(mpu6050, &gyro));

        // Accumulate
        accel_sum[0] += accel.acce_x;
        accel_sum[1] += accel.acce_y;
        accel_sum[2] += accel.acce_z;
        gyro_sum[0] += gyro.gyro_x;
        gyro_sum[1] += gyro.gyro_y;
        gyro_sum[2] += gyro.gyro_z;

        // Delay to maintain ~100Hz
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Compute averages
    sensor_bias.accel_x = accel_sum[0] / samples;
    sensor_bias.accel_y = accel_sum[1] / samples;
    sensor_bias.accel_z = accel_sum[2] / samples - 1.0f; // Subtract 1g (gravity)
    sensor_bias.gyro_x = gyro_sum[0] / samples;
    sensor_bias.gyro_y = gyro_sum[1] / samples;
    sensor_bias.gyro_z = gyro_sum[2] / samples;

    ESP_LOGI("imu",
            "Calibration complete. Biases: Accel (x,y,z) = %.3f, %.3f, %.3f g; Gyro (x,y,z) = %.3f, %.3f, %.3f °/s",
             sensor_bias.accel_x, sensor_bias.accel_y, sensor_bias.accel_z,
             sensor_bias.gyro_x, sensor_bias.gyro_y, sensor_bias.gyro_z);
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

static void
mpu6050_setup(void)
{
    mpu6050 = mpu6050_create(I2C_PORT_NUM, MPU6050_I2C_ADDRESS);
    ESP_ERROR_CHECK(mpu6050_config(mpu6050, ACCE_FS_16G, GYRO_FS_2000DPS));

    uint8_t id = 0;
    ESP_ERROR_CHECK(mpu6050_get_deviceid(mpu6050, &id));

    if (id == MPU6050_WHO_AM_I_VAL) {
        ESP_LOGI("imu", "ID checks out");
    } else {
        ESP_LOGE("imu", "Bad ID: %d", (int)id);
    }
}

static bool IRAM_ATTR
timer_interrupt(gptimer_handle_t timer,
        const gptimer_alarm_event_data_t *eventData, void *userData)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(imu_semaphore, &higher_priority_task_woken);
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

static float yaw = 0.0f;

float
imu_get_yaw(void)
{
    return 0.0f;//yaw;
}

static void
update_yaw(int update_frequency_hz)
{
    mpu6050_gyro_value_t g;
    ESP_ERROR_CHECK(mpu6050_get_gyro(mpu6050, &g));
    mpu6050_acce_value_t a;
    ESP_ERROR_CHECK(mpu6050_get_acce(mpu6050, &a));

    telemetry.raw_imu[0] = a.acce_x;
    telemetry.raw_imu[1] = a.acce_y;
    telemetry.raw_imu[2] = a.acce_z;
    telemetry.raw_imu[3] = g.gyro_x;
    telemetry.raw_imu[4] = g.gyro_y;
    telemetry.raw_imu[5] = g.gyro_z;

    float gyro_z = g.gyro_z - sensor_bias.gyro_z;

    // can't do yaw sensor fusion without a magnetometer. this will drift!!!
    yaw += gyro_z / update_frequency_hz;
    telemetry.yaw = yaw;
}

static void
imu_task(void *arg)
{
    while (1) {
        update_yaw(TIMER_FREQ_HZ);
        xSemaphoreTake(imu_semaphore, portMAX_DELAY);
    }
}

void
imu_init(void)
{
    imu_semaphore = xSemaphoreCreateBinary();
    i2c_setup();
    mpu6050_setup();
    mpu6050_wake_up(mpu6050);
    mpu6050_calibrate();
    xTaskCreate(imu_task, "imu_task", 2048, NULL, 5, NULL);
    timer_setup();
}
