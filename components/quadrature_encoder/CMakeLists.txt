idf_component_register(
    SRCS "quadrature_encoder.c"
    PRIV_REQUIRES esp_driver_gpio esp_timer
    INCLUDE_DIRS "include"
)
