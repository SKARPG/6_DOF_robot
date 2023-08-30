/**
 * @file motor_move.c
 * @author JanG175
 * @brief 6_DOF ROBOT ARM LIBRARY
 * 
 * @copyright All rigths reserved (R) 2023
 */

#include <stdio.h>
#include "motor_move.h"

static const char *TAG = "motor_move";


/**
 * @brief initialize all motors
 * 
 * @param AX_conf struct with AX servo parameters
 * @param emm42_conf struct with emm42 servo parameters
 * @param mks_conf struct with mks servo parameters
 */
void motor_init(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf)
{
    AX_servo_init(AX_conf);
    emm42_servo_init(emm42_conf);
    mks_servo_init(mks_conf);

    // stop all motors
    emm42_servo_uart_move(emm42_conf, 1, 0, 0, 0);
    mks_servo_uart_cr_set_pos(mks_conf, 2, 0, 0, 0);

    vTaskDelay(100 / portTICK_PERIOD_MS);
}


void single_DOF_move(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf, uint8_t DOF, uint32_t position, int16_t speed)
{
    switch (DOF)
    {
        case 0:
            emm42_servo_uart_move(emm42_conf, 0, speed, 0, position);
            break;
        case 1:
            mks_servo_uart_cr_set_pos(mks_conf, 1, speed, 0, position);
            break;
        case 2:
            emm42_servo_uart_move(emm42_conf, 2, speed, 0, position);
            break;
        case 3:
            AX_servo_set_pos(AX_conf, 3, position);
            break;
        case 4:
            AX_servo_set_pos(AX_conf, 4, position);
            break;
        case 5:
            AX_servo_set_pos(AX_conf, 5, position);
            break;
        default:
            ESP_LOGW(TAG, "invalid DOF");
            break;
    }
}