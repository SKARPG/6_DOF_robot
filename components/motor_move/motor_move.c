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
 * @return position in degrees
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
            position = -mks_servo_uart_read_encoder(mks_conf, 2); // CW orientation
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

    // convert servo position to degrees
    if (DOF == 0 || DOF == 1 || DOF == 2)
        position = position / (float)GEAR_RATIO;
    else if (DOF == 3 || DOF == 4 || DOF == 5)
        position = position / 1023.0f * 300.0f;

    return position;
}


/**
 * @brief wait for all motors to stop
 * 
 * @param AX_conf struct with AX servo parameters
 * @param emm42_conf struct with emm42 servo parameters
 * @param mks_conf struct with mks servo parameters
 * @param motor_goal array with goal positions for each motor in degrees
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
            // set acceptable position error
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
    vTaskDelay(25 / portTICK_PERIOD_MS);

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
 * @param position desired position in degrees
 * @param speed_percent percent of speed (0 - 100 %)
 * @param motor_pos array with current positions for each motor in degrees
 */
void single_DOF_move(AX_servo_conf_t AX_conf, emm42_conf_t emm42_conf, mks_conf_t mks_conf, uint8_t DOF, float position, uint8_t speed_percent, float* motor_pos)
{
    uint32_t pulses = 0;
    uint16_t AX_pos = 0;

    int16_t speed = 0;

    if (speed > 100)
    {
        ESP_LOGW(TAG, "too high speed!");
        speed = 100;
    }

    if (DOF == 0 || DOF == 1 || DOF == 2)
    {
        speed = (int16_t)((float)speed_percent / 100.0f * 1279.0f);

        // convert position in degrees to pulses
        pulses = (uint32_t)(fabs(position - motor_pos[DOF]) / 360.0f * (float)FULL_ROT) * GEAR_RATIO;

        // CW orientation
        if (DOF == 1)
        {
            if (position < motor_pos[DOF])
                speed = -speed;
        }
        else
        {
            if (position > motor_pos[DOF])
                speed = -speed;
        }
    }
    else if (DOF == 3 || DOF == 4 || DOF == 5)
    {
        speed = (int16_t)((float)speed_percent / 100.0f * 1023.0f);

        // convert position in degrees to AX servo position
        AX_pos = (uint16_t)(position / 300.0f * 1023.0f);

        if (AX_pos > 1023)
        {
            ESP_LOGW(TAG, "AX servo: too high position!");
            AX_pos = 1023;
        }

        if (speed > 1023)
        {
            ESP_LOGW(TAG, "AX servo: too high speed!");
            speed = 1023;
        }
    }

    switch (DOF)
    {
        case 0:
            emm42_servo_uart_move(emm42_conf, 1, speed, EMM42_ACCEL, pulses);
            break;
        case 1:
            mks_servo_uart_cr_set_pos(mks_conf, 2, speed, MKS_ACCEL, pulses);
            break;
        case 2:
            emm42_servo_uart_move(emm42_conf, 3, speed, EMM42_ACCEL, pulses);
            break;
        case 3:
            AX_servo_set_pos_w_spd(AX_conf, 4, AX_pos, speed);
            break;
        case 4:
            AX_servo_set_pos_w_spd(AX_conf, 5, AX_pos, speed);
            break;
        case 5:
            AX_servo_set_pos_w_spd(AX_conf, 6, AX_pos, speed);
            break;
        default:
            ESP_LOGW(TAG, "invalid DOF!");
            break;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    // update goal position
    if (speed != 0)
    {
        if (DOF == 0 || DOF == 2)
        {
            motor_pos[DOF] -= (float)(speed/abs(speed)) * (float)pulses / (float)FULL_ROT * 360.0f / (float)GEAR_RATIO;
        }
        else if (DOF == 1) // CW orientation
        {
            motor_pos[DOF] += (float)(speed/abs(speed)) * (float)pulses / (float)FULL_ROT * 360.0f / (float)GEAR_RATIO;
        }
        else if (DOF == 3 || DOF == 4 || DOF == 5)
        {
            motor_pos[DOF] = (float)AX_pos / 1023.0f * 300.0f;
        }
    }
}


/**
 * @brief initialize all motors
 * 
 * @param AX_conf struct with AX servo parameters
 * @param emm42_conf struct with emm42 servo parameters
 * @param mks_conf struct with mks servo parameters
 * @param motor_pos array with current positions for each motor
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