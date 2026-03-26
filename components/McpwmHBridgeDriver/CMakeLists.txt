idf_component_register(
    SRCS
        "McpwmHBridgeDriver.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver          # ← le pilote MCPWM legacy est dans ce composant
        esp_rom         # ← pour esp_rom_delay_us()
        IMotorDriver
        AppContext
)