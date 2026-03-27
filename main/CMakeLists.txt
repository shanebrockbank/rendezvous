idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        bsp
        espnow_comms
        gps_nmea
        lcd_i2c
        mpu6050
        hmc5883l
        tasks
        freertos
)
