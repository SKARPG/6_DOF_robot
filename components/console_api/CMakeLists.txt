idf_component_register(SRCS "console_api.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver console fatfs nvs_flash spi_flash vfs motor_move)
