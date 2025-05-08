idf_component_register(
    SRCS "main.c" "gatt_svr.c"
    PRIV_REQUIRES bt nvs_flash
    INCLUDE_DIRS "."
    REQUIRES quadrature_encoder tb6612fng
)
