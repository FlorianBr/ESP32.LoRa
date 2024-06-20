idf_component_register(SRCS "com.c" "main.c" "lcd.c"
    PRIV_REQUIRES lora devicelist app_update nvs_flash
    INCLUDE_DIRS ".")
