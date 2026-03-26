idf_component_register(
    SRCS "gps_nmea.c"
    INCLUDE_DIRS "."
    REQUIRES bsp driver freertos
)
