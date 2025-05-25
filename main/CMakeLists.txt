idf_component_register(
    SRCS "main.c" "ble_telemetry.c" "motors.c" "pid.c" "movement.c" "servos.c"
    PRIV_REQUIRES bt nvs_flash esp_driver_gptimer
    INCLUDE_DIRS "."
    REQUIRES quadrature_encoder tb6612fng
)
