idf_component_register(SRCS "station_example_main.c"
                    PRIV_REQUIRES esp_http_server esp_wifi nvs_flash esp_driver_gpio esp_driver_ledc esp_driver_i2s
                    INCLUDE_DIRS ".")
