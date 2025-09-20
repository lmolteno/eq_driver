idf_component_register(SRCS "hello_world_main.c"
	               SRCS "telescope_protocol.c"
                       SRCS "bluetooth_control.c"
                    PRIV_REQUIRES spi_flash driver esp_timer bt nvs_flash
                    INCLUDE_DIRS "")
