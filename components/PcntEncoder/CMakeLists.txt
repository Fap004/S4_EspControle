idf_component_register(
    SRCS         "PcntEncoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     driver IEncoder        # pour driver/pulse_cnt.h & driver/gpio.h
)