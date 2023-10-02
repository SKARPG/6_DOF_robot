idf_component_register(SRCS "motor_move.c"
                    INCLUDE_DIRS "include"
                    REQUIRES freertos driver log AX_servo emm42_servo mks_servo inv_kin)
