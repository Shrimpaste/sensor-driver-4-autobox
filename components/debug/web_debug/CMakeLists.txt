idf_component_register(
    SRCS "web_debug.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_event
        esp_wifi
        esp_http_server
        esp_netif
        nvs_flash
        spiffs
        esp_timer
)
