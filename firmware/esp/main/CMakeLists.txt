idf_component_register(SRCS "hello_world_main.c"
                            "telescope_protocol.c"
                    PRIV_REQUIRES spi_flash driver esp_timer
                    INCLUDE_DIRS "")
