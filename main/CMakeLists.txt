idf_component_register(
                    SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES com
                    REQUIRES lcd_i2c
                    REQUIRES init
                    )
