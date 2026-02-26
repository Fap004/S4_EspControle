idf_component_register(
                    SRCS "Init.cpp"             # ton fichier source C++
                    INCLUDE_DIRS "include"            # dossier où se trouve lcd_i2c.h
                    REQUIRES driver freertos    # dépendances ESP-IDF utilisées
)