#pragma once

#include "esp_err.h"
#include "stdint.h"

typedef void *tb6612fng_channel_handle_t;

typedef enum {
    TB6612FNG_SHORT_BRAKE = 0,
    TB6612FNG_CCW = 1,
    TB6612FNG_CW = 2,
    TB6612FNG_STOP = 3,
} tb6612fng_channel_drive_t;

/**
 * @brief Initialise TB6612FNG component
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_FAIL: Failed
 */
esp_err_t tb6612fng_init(void);

/**
 * @brief Create and init TB6612FNG channel, return handle
 *
 * @param in1_pin Pin 1 of input (AIN1 or BIN1)
 * @param in2_pin Pin 2 of input (AIN2 or BIN2)
 * @param speed_pin Pin for PWM speed control (PWMA or PWMB)
 * @param ret_handle Returned TB6612FNG channel handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: A paramater is NULL or not valid
 *      - ESP_ERR_NO_MEM: Insufficient memory
 *      - ESP_FAIL: Failed to create TB6612FNG channel
 */
esp_err_t tb6612fng_channel_create(int in1_pin, int in2_pin, int speed_pin,
        tb6612fng_channel_handle_t *ret_handle);

/**
 * @brief Set drive state of TB6612FNG channel
 *
 * @param handle TB6612FNG channel handle
 * @param mode Drive mode (brake, clockwise, counter-clockwise, stop)
 * @param speed Normalised speed in range [0, 1]
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: A paramater is NULL or not valid
 *      - ESP_FAIL: Failed to get encoder count
 */
esp_err_t tb6612fng_channel_set_drive(tb6612fng_channel_handle_t handle, tb6612fng_channel_drive_t mode, float speed);

/**
 * @brief Get drive state of TB6612FNG channel
 *
 * @param handle TB6612FNG channel handle
 * @param ret_mode Returned drive mode (brake, clockwise, counter-clockwise, stop)
 * @param speed Returned normalised speed in range [0, 1]
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: A paramater is NULL or not valid
 *      - ESP_FAIL: Failed to get encoder count
 */
esp_err_t tb6612fng_channel_get_drive(tb6612fng_channel_handle_t handle,
        tb6612fng_channel_drive_t *ret_mode, float *ret_speed);
