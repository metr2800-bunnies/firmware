#include "quadrature_encoder.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_timer.h"

static const char *TAG = "QuadratureEncoder";

typedef struct {
    int channel_a_pin, channel_b_pin;
    volatile int32_t count;
    uint8_t previous_state;
} quadrature_encoder_dev_t;

esp_err_t
quadrature_encoder_init(void)
{
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG, "install gpio isr service failed");
    return ESP_OK;
}

// Lookup table for state transitions (previous_state << 2 | current_state)
// Index: [previous_state:current_state], e.g., 0b0001 is 00 to 01
static const int8_t state_table[16] = {
    0,   1,  -1,  0,  // 00 to 00,01,10,11
   -1,   0,   0,  1,  // 01 to 00,01,10,11
    1,   0,   0, -1,  // 10 to 00,01,10,11
    0,  -1,   1,  0   // 11 to 00,01,10,11
};

static void IRAM_ATTR quadrature_encoder_isr(void *arg)
{
    quadrature_encoder_dev_t *encoder = (quadrature_encoder_dev_t *)arg;
    uint8_t current_state = (gpio_get_level(encoder->channel_a_pin) << 1 | gpio_get_level(encoder->channel_b_pin));
    int8_t delta = state_table[(encoder->previous_state << 2) | current_state];
    encoder->count += delta;
    encoder->previous_state = current_state;
}

esp_err_t
quadrature_encoder_create(int channel_a_pin, int channel_b_pin, quadrature_encoder_handle_t *ret_handle)
{
    esp_err_t ret = ESP_OK;
    quadrature_encoder_dev_t *encoder = (quadrature_encoder_dev_t *)calloc(1, sizeof(quadrature_encoder_dev_t));
    ESP_GOTO_ON_FALSE(encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for quadrature encoder");

    encoder->channel_a_pin = channel_a_pin;
    encoder->channel_b_pin = channel_b_pin;
    encoder->count = 0;
    encoder->previous_state = (
            gpio_get_level(encoder->channel_a_pin) << 1 | gpio_get_level(encoder->channel_b_pin)
    );

    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << channel_a_pin) | (1ULL << channel_b_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_config);

    ESP_GOTO_ON_ERROR(gpio_isr_handler_add(encoder->channel_a_pin, quadrature_encoder_isr, (void *)encoder),
            err, TAG, "add gpio isr handler for encoder channel A failed");
    ESP_GOTO_ON_ERROR(gpio_isr_handler_add(encoder->channel_b_pin, quadrature_encoder_isr, (void *)encoder),
            err, TAG, "add gpio isr handler for encoder channel B failed");

    *ret_handle = encoder;
    return ESP_OK;
err:
    if (encoder != NULL) {
        free(encoder);
    }
    return ret;
}

void
quadrature_encoder_delete(quadrature_encoder_handle_t handle)
{
    quadrature_encoder_dev_t *encoder = (quadrature_encoder_dev_t *)handle;
    if (encoder == NULL) {
        return;
    }
    gpio_isr_handler_remove(encoder->channel_a_pin);
    gpio_isr_handler_remove(encoder->channel_b_pin);
    free(encoder);
}

esp_err_t
quadrature_encoder_get_count(quadrature_encoder_handle_t handle, int32_t *ret_count)
{
    quadrature_encoder_dev_t *encoder = (quadrature_encoder_dev_t *)handle;
    if (encoder == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // FIXME: consider disabling interrupts here to make this atomic
    *ret_count = encoder->count;
    return ESP_OK;
}

esp_err_t
quadrature_encoder_set_count(quadrature_encoder_handle_t handle, int32_t count)
{
    quadrature_encoder_dev_t *encoder = (quadrature_encoder_dev_t *)handle;
    if (encoder == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // FIXME: consider disabling interrupts here to make this atomic
    encoder->count = count;
    return ESP_OK;
}
