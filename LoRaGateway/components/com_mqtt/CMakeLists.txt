idf_component_register(SRCS "mqtt.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash mqtt
)