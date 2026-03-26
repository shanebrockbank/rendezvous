idf_component_register(
    SRCS "espnow_comms.c"
    INCLUDE_DIRS "."
    REQUIRES bsp esp_wifi driver esp_timer freertos
)
