idf_component_register(SRCS "encoder.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_timer esp_driver_pcnt esp_driver_gpio)