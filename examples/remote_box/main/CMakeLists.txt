idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        chassis
        web_control
        esp_driver_ledc
        esp_driver_gpio
        esp_timer
)
