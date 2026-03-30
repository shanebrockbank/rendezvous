idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        bsp
        espnow_comms
        gps_nmea
        lcd_i2c
        icm20948
        tasks
        freertos
)
