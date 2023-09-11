/**
 * @file motor_move.c
 * @author JanG175
 * @brief 6-DOF ROBOT MOTOR SERVO API
 * 
 * @copyright All rigths reserved (R) 2023
 */

#include <stdio.h>
#include "motor_move.h"

static const char *TAG = "motor_move";


/**
 * @brief get motors position
 * 
 * @param AX_conf struct with AX servo parameters
 * @param emm42_conf struct with emm42 servo parameters
 * @param mks_conf struct with mks servo parameters
 * @param DOF number of DOF
 * @return position
 */
float get_motor_pos(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf, uint8_t DOF)
{
    float position = 0.0f;

    switch (DOF)
    {
        case 0:
            position = emm42_servo_uart_read_motor_pos(emm42_conf, 1);
            break;
        case 1:
            position = mks_servo_uart_read_encoder(mks_conf, 2);
            break;
        case 2:
            position = emm42_servo_uart_read_motor_pos(emm42_conf, 3);
            break;
        case 3:
            position = (float)AX_servo_get_pos(AX_conf, 4);
            break;
        case 4:
            position = (float)AX_servo_get_pos(AX_conf, 5);
            break;
        case 5:
            position = (float)AX_servo_get_pos(AX_conf, 6);
            break;
        default:
            ESP_LOGW(TAG, "invalid DOF");
            break;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    return position;
}


/**
 * @brief wait for all motors to stop
 * 
 * @param AX_conf struct with AX servo parameters
 * @param emm42_conf struct with emm42 servo parameters
 * @param mks_conf struct with mks servo parameters
 * @param motor_goal array with goal positions for each motor
 */
void wait_for_motors_stop(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf, float* motor_goal)
{
    bool motor_stop[MOTORS_NUM];
    for (uint32_t i = 0; i < MOTORS_NUM; i++)
        motor_stop[i] = false;

    uint8_t stop = 1;
    float pos_tresh = EMM42_POS_TRESHOLD;

    while (stop)
    {
        for (uint32_t i = 0; i < MOTORS_NUM; i++)
        {
            if (i == 0 || i == 2)
                pos_tresh = EMM42_POS_TRESHOLD;
            else if (i == 1)
                pos_tresh = MKS_POS_TRESHOLD;
            else if (i == 3 || i == 4 || i == 5)
                pos_tresh = AX_POS_TRESHOLD;

            if (fabs(get_motor_pos(AX_conf, emm42_conf, mks_conf, i) - motor_goal[i]) <= pos_tresh)
                motor_stop[i] = true;
            else
                motor_stop[i] = false;

            // printf("%f\n", get_motor_pos(AX_conf, emm42_conf, mks_conf, i) - motor_goal[i]); // for debug
        }

        // printf("\n"); // for debug

        stop = 0;

        for (uint32_t i = 0; i < MOTORS_NUM; i++)
        {
            if (motor_stop[i] == false)
            {
                stop = 1;
                break;
            }
        }
    }

    // prevent position error accumulation
    for (uint32_t i = 0; i < MOTORS_NUM; i++)
        motor_goal[i] = get_motor_pos(AX_conf, emm42_conf, mks_conf, i);
}


/**
 * @brief move each DOF to position with speed
 * 
 * @param AX_conf struct with AX servo parameters
 * @param emm42_conf struct with emm42 servo parameters
 * @param mks_conf struct with mks servo parameters
 * @param DOF number of DOF
 * @param position position
 * @param speed speed
 */
void single_DOF_move(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf, uint8_t DOF, uint32_t position, int16_t speed, float* motor_goal)
{
    uint8_t accel = 0;

    switch (DOF)
    {
        case 0:
            emm42_servo_uart_move(emm42_conf, 1, speed, accel, position);
            break;
        case 1:
            mks_servo_uart_cr_set_pos(mks_conf, 2, speed, accel, position);
            break;
        case 2:
            emm42_servo_uart_move(emm42_conf, 3, speed, accel, position);
            break;
        case 3:
            AX_servo_set_pos(AX_conf, 4, position);
            break;
        case 4:
            AX_servo_set_pos(AX_conf, 5, position);
            break;
        case 5:
            AX_servo_set_pos(AX_conf, 6, position);
            break;
        default:
            ESP_LOGW(TAG, "invalid DOF!");
            break;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (DOF == 0 || DOF == 2)
    {
        if (speed != 0)
            motor_goal[DOF] += (float)(speed/abs(speed)) * (float)position / (float)FULL_ROT * 360.0f;
    }
    else if (DOF == 1)
    {
        if (speed != 0)
            motor_goal[DOF] -= (float)(speed/abs(speed)) * (float)position / (float)FULL_ROT * 360.0f;
    }
    else if (DOF == 3 || DOF == 4 || DOF == 5)
    {
        vTaskDelay(1); // TODO !!!
    }
}


/**
 * @brief initialize all motors
 * 
 * @param AX_conf struct with AX servo parameters
 * @param emm42_conf struct with emm42 servo parameters
 * @param mks_conf struct with mks servo parameters
 */
void motor_init(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf, float* motor_pos)
{
    AX_servo_init(AX_conf);
    emm42_servo_init(emm42_conf);
    mks_servo_init(mks_conf);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    // stop all motors
    for (uint32_t i = 0; i < MOTORS_NUM; i++)
    {
        single_DOF_move(AX_conf, emm42_conf, mks_conf, i, 0, 0, motor_pos);
        motor_pos[i] = get_motor_pos(AX_conf, emm42_conf, mks_conf, i);
    }
}