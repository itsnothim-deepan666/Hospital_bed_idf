idf_component_register(SRCS "main.c" 
                    INCLUDE_DIRS "."
                    REQUIRES micro_ros_espidf_component
                    PRIV_REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_uart)
