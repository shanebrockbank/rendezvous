idf_component_register(
    SRCS "tasks.c"
    INCLUDE_DIRS "."
    REQUIRES
        bsp
        espnow_comms
        lcd_i2c
        mpu6050
        hmc5883l
        gps_nmea
        rendezvous_logic
        driver
        esp_timer
        freertos
)
