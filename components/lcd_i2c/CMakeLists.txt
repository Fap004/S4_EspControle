idf_component_register(
                    SRCS "Lcd_I2C.cpp"        # ton fichier source C++
                    INCLUDE_DIRS "."          # dossier où se trouve lcd_i2c.h
                    REQUIRES driver freertos  # dépendances ESP-IDF utilisées
)