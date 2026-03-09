idf_component_register(
    SRCS         "PcntEncoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_pcnt esp_driver_gpio IEncoder        # pour driver/pulse_cnt.h & driver/gpio.h
)