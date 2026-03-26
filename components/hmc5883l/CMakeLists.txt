idf_component_register(
    SRCS "hmc5883l.c"
    INCLUDE_DIRS "."
    REQUIRES bsp driver freertos
)
