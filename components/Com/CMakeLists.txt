idf_component_register(
    SRCS
        "Com.cpp"
    INCLUDE_DIRS
        "include"

    REQUIRES
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
)
