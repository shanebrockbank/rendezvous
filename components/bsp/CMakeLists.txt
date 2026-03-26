idf_component_register(
    SRCS "bsp.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash driver esp_timer
)
