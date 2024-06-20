idf_component_register(SRCS "wifi.c" "ntp.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_wifi
)