idf_component_register(
    SRCS
        "main.c"
        "eth_transport.c"
        "pca9685.c"
        "webserver.c"
        "system_stats.c"
        "radio.c"
        "rgb_led.c"
        "ultrasonic.c"
        "ssd1306.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_eth
        esp_netif
        esp_event
        esp_http_server
        esp_timer
        nvs_flash
        json
        freertos
        esp_driver_i2c
        esp_driver_i2s
        esp_driver_tsens
        esp_driver_gpio
        esp_driver_sdmmc
        spi_flash
        fatfs
        vfs
        micro_ros_espidf_component
)

# Allow multiple definitions for 64-bit atomics (rcutils vs newlib)
target_link_options(${COMPONENT_LIB} INTERFACE -Wl,--allow-multiple-definition)
