idf_component_register(
    SRCS "MCpwmServo.cpp" "McpwmServo.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_gpio IServo
)