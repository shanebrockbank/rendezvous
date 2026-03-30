idf_component_register(
    SRCS "tasks.c"
    INCLUDE_DIRS "."
    REQUIRES
        bsp
        espnow_comms
        lcd_i2c
        icm20948
        gps_nmea
        rendezvous_logic
        distance_estimator
        driver
        esp_timer
        freertos
)
