#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include "stdint.h"

typedef void *quadrature_encoder_handle_t;

/**
 * @brief Initialise the quadrature encoder component
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_NO_MEM: No memory to install this service
 *      - ESP_ERR_INVALID_STATE: ISR service already installed.
 *      - ESP_ERR_NOT_FOUND: No free interrupt found with the specified flags
 *      - ESP_ERR_INVALID_ARG: GPIO error
 */

esp_err_t quadrature_encoder_init(void);

/**
 * @brief Create and init quadrature encoder, return handle
 *
 * @param channel_a_pin Pin for encoder channel A
 * @param channel_b_pin Pin for encoder channel B
 * @param ret_handle Returned quadrature encoder handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: A paramater is NULL or not valid
 *      - ESP_ERR_NO_MEM: Insufficient memory
 *      - ESP_FAIL: Failed to create encoder
 */
esp_err_t quadrature_encoder_create(int channel_a_pin, int channel_b_pin,
        quadrature_encoder_handle_t *ret_handle);

/**
 * @brief Delete and release a quadrature encoder
 *
 * @param handle Quadrature encoder handle
 */
void quadrature_encoder_delete(quadrature_encoder_handle_t handle);

/**
 * @brief Get encoder count
 *
 * @param handle Quadrature encoder handle
 * @param ret_count Returned encoder count
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: A paramater is NULL or not valid
 *      - ESP_FAIL: Failed to get encoder count
 */
esp_err_t quadrature_encoder_get_count(quadrature_encoder_handle_t handle, int32_t *ret_count);

/**
 * @brief Set encoder count
 *
 * @param handle Quadrature encoder handle
 * @param count Encoder count
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: A paramater is NULL or not valid
 *      - ESP_FAIL: Failed to set encoder count
 */
esp_err_t quadrature_encoder_set_count(quadrature_encoder_handle_t handle, int32_t count);
