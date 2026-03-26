idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        bsp
        espnow_comms
        gps_nmea
        tasks
        freertos
)
