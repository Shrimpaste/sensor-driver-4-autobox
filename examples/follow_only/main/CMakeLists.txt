idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        bu_uwb
        imu_i2c
        chassis
        esp_driver_i2c
        esp_timer
        esp_driver_ledc
        esp_driver_gpio
        esp_event
        esp_wifi
        esp_http_server
        esp_netif
        nvs_flash
        spiffs
)
